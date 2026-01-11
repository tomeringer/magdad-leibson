#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ["PIGPIO_ADDR"] = "localhost"

import time
import socket
import pigpio
import RPi.GPIO as GPIO
from typing import Optional
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# ============================================================
# CONFIGURATION
# ============================================================
MODE = "GLOVE"

UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

factory = PiGPIOFactory()

# ============================================================
# PIN MAP (Pi)  — BCM numbering + Physical pin for sanity
# ============================================================
# Servo:
#   SERVO_PIN = BCM12  (Physical pin 32)
#
# Ultrasonic:
#   TRIG = BCM5  (Physical pin 29)
#   ECHO = BCM6  (Physical pin 31)
#
# DC Motors (IBT-2 style, 2 PWM per motor):
#   RIGHT_RPWM = BCM2  (Physical pin 3)
#   RIGHT_LPWM = BCM3  (Physical pin 5)
#   LEFT_RPWM  = BCM20 (Physical pin 38)
#   LEFT_LPWM  = BCM21 (Physical pin 40)
#
# Stepper (4-wire via driver like ULN2003/L298N full-step sequence):
#   STEPPER_PINS = [BCM23, BCM22, BCM27, BCM17]
#                = [Phys 16, Phys 15, Phys 13, Phys 11]
#
# ============================================================
# GLOVE BYTE MAP (matches your ESP32 code)
# ============================================================
# ESP32 builds:
#   dataByte = (flexBits << 4) | (rollBits << 2) | pitchBits
#
# flexBits is built from flexPins array:
#   flexPins = {A0, A2, A1, A3}
# so the bits mean:
#   bit0 (f0) = A0
#   bit1 (f1) = A2
#   bit2 (f2) = A1
#   bit3 (f3) = A3
#
# rollBits:
#   01 => roll  > +ROLL_TH
#   10 => roll  < -ROLL_TH
#
# pitchBits:
#   01 => pitch > +PITCH_TH   (Forward)
#   10 => pitch < -PITCH_TH   (Reverse)
# ============================================================

# ============================================================
# ULTRASONIC SENSOR
# ============================================================
TRIG, ECHO = 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ULTRA_STOP_CM = 40.0
_last_distance_cm: Optional[float] = None

def measure_distance_cm() -> Optional[float]:
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > 0.03:
            return None

    ps = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - ps > 0.03:
            return None

    pe = time.time()
    return (pe - ps) * 17150.0


# ============================================================
# SERVO - Step Logic with Clamping
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo: Optional[Servo] = None

def servo_init():
    global _servo
    _servo = Servo(
        SERVO_PIN,
        initial_value=None,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
    )
    print(f"[SERVO] Ready on GPIO {SERVO_PIN}")

def servo_move_step(direction: int):
    """
    direction: 1 for minus step, 0 for plus step
    """
    global current_pos
    if _servo is None:
        return

    new_val = (current_pos - MOVE_STEP) if direction == 1 else (current_pos + MOVE_STEP)
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos
    print(f"[SERVO] Edge Detected! Moved to {current_pos:.2f}")


# ============================================================
# DC MOTORS (PWM via pigpio factory)
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2,  frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3,  frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM  = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM  = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)

def dc_stop():
    RIGHT_RPWM.value = 0.0
    RIGHT_LPWM.value = 0.0
    LEFT_RPWM.value  = 0.0
    LEFT_LPWM.value  = 0.0


# ============================================================
# STEPPER - Non-blocking "continuous while held"
# ============================================================
STEPPER_PINS = [23, 22, 27, 17]

STEPPER_STEP_INTERVAL_SEC = 0.0015   # smaller = faster
STEPPER_MAX_STEPS_PER_UPDATE = 50    # cap CPU per loop
STEPPER_IDLE_COIL_OFF_SEC = 0.20     # turn coils off after idle to reduce heat

class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not connected. Run: sudo systemctl enable --now pigpiod")

        # Standard 4-step full-step sequence:
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 0, 1],
        ]

        for p in self.pins:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.write(p, 0)

        self._seq_idx = 0
        self._dir = 0  # -1, 0, +1
        self._last_step_t = time.monotonic()
        self._last_active_t = time.monotonic()

    def set_direction(self, direction: int):
        # direction in {-1, 0, +1}
        if direction not in (-1, 0, 1):
            direction = 0
        self._dir = direction
        if direction != 0:
            self._last_active_t = time.monotonic()

    def _apply_state(self, idx: int):
        state = self.seq[idx]
        for i, p in enumerate(self.pins):
            self.pi.write(p, state[i])

    def coils_off(self):
        for p in self.pins:
            self.pi.write(p, 0)

    def update(self):
        now = time.monotonic()

        # If idle, optionally release coils after a short timeout
        if self._dir == 0:
            if (now - self._last_active_t) > STEPPER_IDLE_COIL_OFF_SEC:
                self.coils_off()
            return

        # Step as many times as elapsed allows, capped
        elapsed = now - self._last_step_t
        steps_to_do = int(elapsed / STEPPER_STEP_INTERVAL_SEC)
        if steps_to_do <= 0:
            return
        if steps_to_do > STEPPER_MAX_STEPS_PER_UPDATE:
            steps_to_do = STEPPER_MAX_STEPS_PER_UPDATE

        for _ in range(steps_to_do):
            self._seq_idx = (self._seq_idx + self._dir) % len(self.seq)
            self._apply_state(self._seq_idx)

        # advance last_step_t by the exact stepped time to keep stable rate
        self._last_step_t += steps_to_do * STEPPER_STEP_INTERVAL_SEC
        self._last_active_t = now

_stepper = StepperMotor(STEPPER_PINS)


# ============================================================
# PAYLOAD HANDLER - EDGE DETECTION (servo) + HOLD (stepper)
# ============================================================
prev_f0 = 0
prev_f1 = 0

def handle_payload(payload: int):
    global prev_f0, prev_f1, _last_distance_cm

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    # f0..f3 map to ESP32 flexPins {A0, A2, A1, A3} respectively
    f0 = (flex >> 0) & 1  # A0
    f1 = (flex >> 1) & 1  # A2
    f2 = (flex >> 2) & 1  # A1
    f3 = (flex >> 3) & 1  # A3

    # --- Servo: edge detect (0->1) ---
    if f0 == 1 and prev_f0 == 0:
        print("[EVENT] f0 (A0) pressed -> Closing Servo")
        servo_move_step(1)

    if f1 == 1 and prev_f1 == 0:
        print("[EVENT] f1 (A2) pressed -> Opening Servo")
        servo_move_step(0)

    prev_f0, prev_f1 = f0, f1

    # --- Stepper: continuous while held ---
    # choose one direction; if both held (or none), stop
    step_dir = 0
    if f2 == 1 and f3 == 0:
        step_dir = 1
    elif f3 == 1 and f2 == 0:
        step_dir = -1
    else:
        step_dir = 0
    _stepper.set_direction(step_dir)

    # --- Ultrasonic + DC motors ---
    _last_distance_cm = measure_distance_cm()
    too_close = (_last_distance_cm is not None and _last_distance_cm < ULTRA_STOP_CM)

    if pitch_code == 0b01 and not too_close:  # Forward
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0

    elif pitch_code == 0b10:  # Reverse
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0

    elif roll_code == 0b01 and not too_close:  # Right
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0

    elif roll_code == 0b10 and not too_close:  # Left
        RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0

    else:
        dc_stop()


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.1)
    print(f"[RUNNING] Port {UDP_PORT} | Servo=edge | Stepper=continuous-hold")

    last_rx = time.time()
    while True:
        # Always keep stepper "alive" even between packets
        _stepper.update()

        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()

        except socket.timeout:
            # stop DC motors on silence, and also stop stepper
            if time.time() - last_rx > SILENCE_STOP_SEC:
                dc_stop()
                _stepper.set_direction(0)

        time.sleep(CONTROL_PERIOD_SEC)


if __name__ == "__main__":
    servo_init()
    try:
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        dc_stop()
        _stepper.set_direction(0)
        _stepper.coils_off()
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")
