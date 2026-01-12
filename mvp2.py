#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['PIGPIO_ADDR'] = 'localhost'

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

# ---- Stepper "continuous feel" ----
STEPPER_CHUNK = 4                 # keep small so stop feels immediate
STEPPER_STEP_DELAY_SEC = 0.0005   # was 0.0015
STEPPER_TICK_SEC = 0.05           # glove sends every 50ms -> match it
_last_stepper_cmd_t = 0.0

# ---- Stepper command freshness gate (Choice #1) ----
STEP_CMD_STALE_SEC = 0.12         # if no valid packets for this long -> stop stepper immediately
_stepper_dir = 0                  # -1, 0, +1 (desired direction while held)
_last_pkt_t = 0.0                 # last time we received a valid framed packet

# ============================================================
# GLOBAL STATE (edge-detected)
# ============================================================
prev_f0 = prev_f1 = 0

# ============================================================
# ULTRASONIC SENSORS (ASYNC)
# ============================================================
US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(US1_TRIG, GPIO.OUT)
GPIO.setup(US1_ECHO, GPIO.IN)
GPIO.setup(US2_TRIG, GPIO.OUT)
GPIO.setup(US2_ECHO, GPIO.IN)

GPIO.output(US1_TRIG, GPIO.LOW)
GPIO.output(US2_TRIG, GPIO.LOW)

ULTRA_STOP_CM = 40.0
US_TIMEOUT_SEC = 0.03
ULTRA_MEASURE_PERIOD_SEC = 0.10

_last_ultra_t = 0.0
_last_distances = [None, None]


def _measure_distance_cm(trig, echo) -> Optional[float]:
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    t0 = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - t0 > US_TIMEOUT_SEC:
            return None

    ps = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - ps > US_TIMEOUT_SEC:
            return None

    pe = time.time()
    return (pe - ps) * 17150.0


def ultrasonic_tick():
    global _last_ultra_t, _last_distances
    now = time.time()
    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        _last_distances = [
            _measure_distance_cm(US1_TRIG, US1_ECHO),
            _measure_distance_cm(US2_TRIG, US2_ECHO),
        ]


def obstacle_too_close() -> bool:
    for d in _last_distances:
        if d is not None and d < ULTRA_STOP_CM:
            return True
    return False


# ============================================================
# SERVO
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo = None


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
    global current_pos
    if _servo is None:
        return

    new_val = current_pos - MOVE_STEP if direction == 1 else current_pos + MOVE_STEP
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos
    print(f"[SERVO] Edge Detected! Moved to {current_pos:.2f}")


# ============================================================
# DC MOTORS
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM  = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM  = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


def stop_dc():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


# ============================================================
# STEPPER MOTOR
# ============================================================
class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
        ]
        for p in self.pins:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.write(p, 0)

    def step_chunk(self, steps: int, direction: int = 1, delay_sec: float = 0.002):
        steps = int(max(0, steps))
        direction = 1 if direction >= 0 else -1

        for _ in range(steps):
            for step in range(4):
                idx = step if direction == 1 else 3 - step
                for i, p in enumerate(self.pins):
                    self.pi.write(p, self.seq[idx][i])
                time.sleep(delay_sec)

        # de-energize between chunks
        self.deenergize()

    def deenergize(self):
        for p in self.pins:
            self.pi.write(p, 0)

    def close(self):
        self.deenergize()
        try:
            self.pi.stop()
        except Exception:
            pass


_stepper = StepperMotor([23, 22, 27, 17])


def stepper_tick(direction: int):
    global _last_stepper_cmd_t
    now = time.time()
    if now - _last_stepper_cmd_t >= STEPPER_TICK_SEC:
        _last_stepper_cmd_t = now
        _stepper.step_chunk(STEPPER_CHUNK, direction=direction, delay_sec=STEPPER_STEP_DELAY_SEC)


# ============================================================
# PAYLOAD HANDLER (MATCHES YOUR ESP32 PACKET ENCODING)
#   dataByte = (flexBits<<4) | (rollBits<<2) | pitchBits
#
#   flexPins order on ESP32: {A0, A2, A1, A3}
#   so:
#     f0 -> bit0 -> A0
#     f1 -> bit1 -> A2
#     f2 -> bit2 -> A1
#     f3 -> bit3 -> A3
# ============================================================
def handle_payload(payload: int):
    global prev_f0, prev_f1, _stepper_dir, _last_pkt_t

    _last_pkt_t = time.time()  # refresh stepper-command validity on every valid packet

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # Servo edge detection
    if f0 == 1 and prev_f0 == 0:
        print("[EVENT] Finger 0 (A0) pressed -> Closing Servo")
        servo_move_step(1)

    if f1 == 1 and prev_f1 == 0:
        print("[EVENT] Finger 1 (A2) pressed -> Opening Servo")
        servo_move_step(0)

    prev_f0, prev_f1 = f0, f1

    # Stepper "held" command: only set desired direction (movement happens in main loop)
    if f2 == 1 and f3 == 0:
        _stepper_dir = +1
    elif f3 == 1 and f2 == 0:
        _stepper_dir = -1
    else:
        _stepper_dir = 0

    # DC motors (unchanged logic)
    too_close = obstacle_too_close()

    if pitch_code == 0b01 and not too_close:
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_LPWM.value = LEFT_LPWM.value = 0.0

    elif pitch_code == 0b10:
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_RPWM.value = LEFT_RPWM.value = 0.0

    elif roll_code == 0b01 and not too_close:
        RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_LPWM.value = LEFT_RPWM.value = 0.0

    elif roll_code == 0b10 and not too_close:
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_RPWM.value = LEFT_LPWM.value = 0.0

    else:
        stop_dc()


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    global _stepper_dir, _last_pkt_t

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.02)  # faster silence detection

    print(f"[RUNNING] Glove UDP + Async Ultrasonic on Port {UDP_PORT}")

    last_rx = time.time()
    _last_pkt_t = time.time()  # initialize to "now" so we don't instantly stale-stop on startup

    while True:
        ultrasonic_tick()

        # ----- Stepper freshness gate: if packets stop, stop stepper immediately -----
        now = time.time()
        if now - _last_pkt_t > STEP_CMD_STALE_SEC:
            if _stepper_dir != 0:
                _stepper_dir = 0
                _stepper.deenergize()  # immediate stop (drop coils)

        # ----- Stepper continuous motion only while command is fresh -----
        if _stepper_dir != 0:
            stepper_tick(_stepper_dir)

        # ----- UDP receive -----
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            # Stop DC motors on long silence
            if time.time() - last_rx > SILENCE_STOP_SEC:
                stop_dc()

        time.sleep(CONTROL_PERIOD_SEC)


# ============================================================
# PIN MAPPING (Raspberry Pi, BCM numbering)
#
#   Usage                  BCM   Physical pin
#   ------------------------------------------------
#   Ultrasonic #1 TRIG      5    pin 29
#   Ultrasonic #1 ECHO      6    pin 31
#   Ultrasonic #2 TRIG     13    pin 33
#   Ultrasonic #2 ECHO     19    pin 35
#
#   Servo signal           12    pin 32
#
#   Right DC motor RPWM     2    pin 3
#   Right DC motor LPWM     3    pin 5
#   Left  DC motor RPWM    20    pin 38
#   Left  DC motor LPWM    21    pin 40
#
#   Stepper IN1            23    pin 16
#   Stepper IN2            22    pin 15
#   Stepper IN3            27    pin 13
#   Stepper IN4            17    pin 11
# ============================================================


# ============================================================
# ENTRY
# ============================================================
if __name__ == "__main__":
    servo_init()
    try:
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_dc()
        try:
            _stepper.close()
        except Exception:
            pass
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")
