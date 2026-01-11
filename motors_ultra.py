#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import socket
import pigpio
import RPi.GPIO as GPIO
from typing import Optional
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# ×”×’×“×¨×ª ×›×ª×•×‘×ª ×œ-pigpio
os.environ['PIGPIO_ADDR'] = 'localhost'

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

# ×ž×©×ª× ×™× ×’×œ×•×‘×œ×™×™× ×œ×ž×¦×‘ ×”×ž×¢×¨×›×ª
prev_f0 = 0
prev_f1 = 0
prev_f2 = 0
prev_f3 = 0
_current_motion = "stop"

# ============================================================
# ULTRASONIC SENSORS SETUP
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

US_STOP_DISTANCE_CM = 50.0
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
        if time.time() - t0 > US_TIMEOUT_SEC: return None

    pulse_start = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - pulse_start > US_TIMEOUT_SEC: return None

    pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150.0


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
        if d is not None and d < US_STOP_DISTANCE_CM:
            return True
    return False


# ============================================================
# SERVO - Step Logic
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo = None


def servo_init():
    global _servo
    _servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory,
                   min_pulse_width=0.5 / 1000, max_pulse_width=2.5 / 1000)
    print(f"[SERVO] Ready on GPIO {SERVO_PIN}")


def servo_move_step(direction: int):
    global current_pos
    if _servo is None: return
    new_val = current_pos - MOVE_STEP if direction == 1 else current_pos + MOVE_STEP
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos


# ============================================================
# DC & STEPPER MOTORS
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


def motor_stop():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [[1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]]
        for p in self.pins: self.pi.set_mode(p, pigpio.OUTPUT)

    def move(self, steps, direction=1):
        for _ in range(int(steps)):
            for step in range(len(self.seq)):
                idx = step if direction == 1 else (3 - step)
                for i, p in enumerate(self.pins): self.pi.write(p, self.seq[idx][i])
                time.sleep(0.005)
        for p in self.pins: self.pi.write(p, 0)


_stepper = StepperMotor([23, 22, 27, 17])


# ============================================================
# PAYLOAD HANDLER
# ============================================================
def handle_payload(payload: int):
    global prev_f0, prev_f1, prev_f2, prev_f3, _current_motion

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    f0, f1, f2, f3 = [(flex >> i) & 1 for i in range(4)]

    if f0 == 1 and prev_f0 == 0: servo_move_step(1)
    if f1 == 1 and prev_f1 == 0: servo_move_step(0)
    prev_f0, prev_f1 = f0, f1

    if f2 == 1 and prev_f2 == 0: _stepper.move(50, 1)
    if f3 == 1 and prev_f3 == 0: _stepper.move(50, -1)
    prev_f2, prev_f3 = f2, f3

    too_close = obstacle_too_close()
    desired = "stop"
    if pitch_code == 0b01:
        desired = "forward"
    elif pitch_code == 0b10:
        desired = "reverse"
    elif roll_code == 0b01:
        desired = "turn_right"
    elif roll_code == 0b10:
        desired = "turn_left"

    if desired in ("forward", "turn_left", "turn_right") and too_close:
        motor_stop()
        _current_motion = "stop"
        return

    if desired == "forward":
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0
    elif desired == "reverse":
        RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0
    elif desired == "turn_right":
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
        RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0
    elif desired == "turn_left":
        RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
        RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0
    else:
        motor_stop()

    _current_motion = desired


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    global _current_motion  # ×”×•×¡×¤×ª ×”×¦×”×¨×” ×’×œ×•×‘×œ×™×ª ×œ×ª×™×§×•×Ÿ ×”×©×’×™××”

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.05)
    print(f"[RUNNING] Safe Mode (Ultrasonic Enabled) on Port {UDP_PORT}")

    last_rx = time.time()
    while True:
        ultrasonic_tick()

        if _current_motion in ("forward", "turn_left", "turn_right") and obstacle_too_close():
            motor_stop()
            _current_motion = "stop"
            print("[ULTRA][EMERGENCY STOP] Obstacle detected!")

        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC:
                motor_stop()
                _current_motion = "stop"

        time.sleep(CONTROL_PERIOD_SEC)


if __name__ == "__main__":
    servo_init()
    try:
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        motor_stop()
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")