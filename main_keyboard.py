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
MODE = "KEYBOARD"
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

factory = PiGPIOFactory()

# משתנים לשמירת מצב קודם של האצבעות (לזיהוי שינוי מ-0 ל-1)
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
# HELPERS (match your existing behavior)
# ============================================================
def stop_drive():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


def drive_forward():
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.51, 0.50
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0


def drive_reverse():
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0


def turn_right():
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0


def turn_left():
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0


# ============================================================
# KEYBOARD LOOP (replaces UDP glove loop)
# ============================================================
def run_keyboard_loop():
    global _last_distance_cm

    print("\n[READY] w/s/a/d=Drive, x=Stop, i/k=Servo, u/j=Stepper, q=Quit")
    print("        (Ultrasonic safety: forward/turn disabled if too close)\n")

    while True:
        try:
            cmd = input("cmd> ").strip().lower()
            if not cmd:
                continue
            c = cmd[0]

            if c == 'q':
                print("[QUIT]")
                break

            # Measure distance each command (same as handle_payload)
            _last_distance_cm = measure_distance_cm()
            too_close = (_last_distance_cm is not None and _last_distance_cm < ULTRA_STOP_CM)

            if _last_distance_cm is None:
                print("[ULTRA] distance=None (timeout)")
            else:
                print(f"[ULTRA] distance={_last_distance_cm:.1f} cm (stop<{ULTRA_STOP_CM:.1f})")

            if c == 'x':
                stop_drive()
                print("[STOP] Drive stopped")

            elif c == 'w':  # Forward (blocked by ultrasonic)
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop")
                else:
                    drive_forward()
                    print("[DRIVE] Forward")

            elif c == 's':  # Reverse (allowed even if close)
                drive_reverse()
                print("[DRIVE] Reverse")

            elif c == 'a':  # Left (blocked by ultrasonic)
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop")
                else:
                    turn_left()
                    print("[TURN] Left")

            elif c == 'd':  # Right (blocked by ultrasonic)
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop")
                else:
                    turn_right()
                    print("[TURN] Right")

            elif c == 'i':  # Servo close (same as finger0 rising edge)
                print("[SERVO] Close step")
                servo_move_step(1)

            elif c == 'k':  # Servo open (same as finger1 rising edge)
                print("[SERVO] Open step")
                servo_move_step(0)

            elif c == 'u':  # Stepper +50 (same as finger2 rising edge)
                print("[STEPPER] +50")
                _stepper.move(50, 1)

            elif c == 'j':  # Stepper -50 (same as finger3 rising edge)
                print("[STEPPER] -50")
                _stepper.move(50, -1)

            else:
                print("[INFO] Unknown command. Use w/s/a/d/x/i/k/u/j/q")

        except UnicodeDecodeError:
            print("[ERROR] Switch keyboard to ENGLISH!")
        except KeyboardInterrupt:
            print("\n[QUIT] KeyboardInterrupt")
            break
        except Exception as e:
            print(f"[ERROR] {e}")

    # Safety on exit
    stop_drive()


if __name__ == "__main__":
    servo_init()
    try:
        run_keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")