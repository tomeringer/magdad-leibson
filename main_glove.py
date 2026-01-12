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

# ?????? ?????? ??? ???? ?? ??????? (?????? ????? ?-0 ?-1)
prev_f0 = 0
prev_f1 = 0
prev_f2 = 0
prev_f3 = 0

# ============================================================
# ULTRASONIC SENSOR
# ============================================================
TRIG, ECHO = 5, 6
GPIO.setmode(GPIO.BCM)
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
        if time.time() - t0 > 0.03: return None
    ps = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - ps > 0.03: return None
    pe = time.time()
    return (pe - ps) * 17150.0


# ============================================================
# SERVO - Step Logic with Clamping
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
    """
    direction: 1 for minus step, 0 for plus step
    """
    global current_pos
    if _servo is None: return

    if direction == 1:
        new_val = current_pos - MOVE_STEP
    else:
        new_val = current_pos + MOVE_STEP

    # Clamping
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos
    print(f"[SERVO] Edge Detected! Moved to {current_pos:.2f}")


# ============================================================
# DC & STEPPER MOTORS
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


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
# PAYLOAD HANDLER - EDGE DETECTION LOGIC
# ============================================================
def handle_payload(payload: int):
    global prev_f0, prev_f1, prev_f2, prev_f3, _last_distance_cm

    # ????? ?????
    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    # ??? ????? ?? ??????
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # --- ?????? ???? (????? ???? ?-0 ?-1) ---
    if f0 == 1 and prev_f0 == 0:
        print("[EVENT] Finger 0 pressed -> Closing Servo")
        servo_move_step(1)  # ????? ???

    if f1 == 1 and prev_f1 == 0:
        print("[EVENT] Finger 1 pressed -> Opening Servo")
        servo_move_step(0)  # ????? ???

    # ????? ??? ???? ???? ???
    prev_f0, prev_f1 = f0, f1

    # --- ?????? ???? (????? ???? ?-0 ?-1) ---
    if f2 == 1 and prev_f2 == 0:
        _stepper.move(STEPPER_CHUNK, 1)
    if f3 == 1 and prev_f3 == 0:
        _stepper.move(STEPPER_CHUNK, -1)

    prev_f2, prev_f3 = f2, f3

    # --- ?????? ????? (DC Motors) ---
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
        RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.1)
    print(f"[RUNNING] Edge Detection Mode on Port {UDP_PORT}")

    last_rx = time.time()
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC:
                RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0
        time.sleep(CONTROL_PERIOD_SEC)


if __name__ == "__main__":
    servo_init()
    try:
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")