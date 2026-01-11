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
STEPPER_CHUNK = 200

# ============================================================
# FLEX STATE (EDGE DETECTION)
# ============================================================
prev_step_up = 0
prev_step_down = 0
prev_servo_close = 0
prev_servo_open = 0

# ============================================================
# ULTRASONIC (UNCHANGED)
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
    return any(d is not None and d < ULTRA_STOP_CM for d in _last_distances)

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


def servo_move_step(close: bool):
    global current_pos
    delta = -MOVE_STEP if close else MOVE_STEP
    current_pos = max(-1.0, min(1.0, current_pos + delta))
    _servo.value = current_pos
    print(f"[SERVO] Moved to {current_pos:.2f}")

# ============================================================
# MOTORS
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM  = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM  = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


class StepperMotor:
    def init(self, pins):
        self.pi = pigpio.pi()
        self.pins = pins
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
        ]
        for p in pins:
            self.pi.set_mode(p, pigpio.OUTPUT)

def move(self, steps, direction):
        for _ in range(steps):
            for i in range(4):
                idx = i if direction > 0 else 3 - i
                for p, v in zip(self.pins, self.seq[idx]):
                    self.pi.write(p, v)
                time.sleep(0.005)
        for p in self.pins:
            self.pi.write(p, 0)


_stepper = StepperMotor([23, 22, 27, 17])

# ============================================================
# PAYLOAD HANDLER (UPDATED MAPPING)
# ============================================================
def handle_payload(payload: int):
    global prev_step_up, prev_step_down, prev_servo_close, prev_servo_open

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    step_up       = (flex >> 0) & 1
    step_down     = (flex >> 1) & 1
    servo_close   = (flex >> 2) & 1
    servo_open    = (flex >> 3) & 1

    # --- EDGE DETECTION ---
    if step_up and not prev_step_up:
        print("[STEP] UP")
        _stepper.move(STEPPER_CHUNK, +1)

    if step_down and not prev_step_down:
        print("[STEP] DOWN")
        _stepper.move(STEPPER_CHUNK, -1)

    if servo_close and not prev_servo_close:
        print("[SERVO] CLOSE")
        servo_move_step(close=True)

    if servo_open and not prev_servo_open:
        print("[SERVO] OPEN")
        servo_move_step(close=False)

    prev_step_up = step_up
    prev_step_down = step_down
    prev_servo_close = servo_close
    prev_servo_open = servo_open

    # --- DRIVE (UNCHANGED) ---
    too_close = obstacle_too_close()

    if pitch_code == 0b01 and not too_close:
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_LPWM.value = LEFT_LPWM.value = 0.0
    elif pitch_code == 0b10:
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_RPWM.value = LEFT_RPWM.value = 0.0
    elif roll_code == 0b01 and not too_close:
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_RPWM.value = LEFT_LPWM.value = 0.0
    elif roll_code == 0b10 and not too_close:
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_LPWM.value = LEFT_RPWM.value = 0.0
    else:
        RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0

# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.1)

    print(f"[RUNNING] Glove Receiver on UDP {UDP_PORT}")

    last_rx = time.time()
    while True:
        ultrasonic_tick()
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC:
                RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0
        time.sleep(CONTROL_PERIOD_SEC)

# ============================================================
# ENTRY
# ============================================================
if name == "main":
    servo_init()
    try:
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")