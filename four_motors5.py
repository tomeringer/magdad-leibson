#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

# חובה להשאיר בראש הקובץ לחיבור תקין מרחוק
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
import pigpio
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# הגדרת ה-Factory המשותף למנועי ה-DC והסרוו
factory = PiGPIOFactory()

# ============================================================
# SERVO - לוגיקה מהקוד השני, פין 12
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo = None


def servo_init():
    global _servo
    _servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory,
                   min_pulse_width=0.5 / 1000, max_pulse_width=2.5 / 1000)
    print(f"[SERVO] Initialized on GPIO {SERVO_PIN} at pos 0.0")


def servo_spin(direction_bit: int):
    global current_pos
    if _servo is None: return

    # לוגיקה שביקשת: 1 (i) מחסיר ערך, 0 (k) מוסיף ערך
    if direction_bit:
        new_val = current_pos - MOVE_STEP
    else:
        new_val = current_pos + MOVE_STEP

    # הגנה על גבולות המנוע
    if new_val > 1.0: new_val = 1.0
    if new_val < -1.0: new_val = -1.0

    current_pos = new_val
    _servo.value = current_pos
    print(f"[SERVO] Moved to {current_pos:.2f}")


def servo_cleanup():
    if _servo:
        _servo.detach()


# ============================================================
# DC MOTORS (IBT-2)
# ============================================================
PWM_HZ = 1000
# הגדרת פינים (BCM)
RIGHT_RPWM = PWMOutputDevice(2, frequency=PWM_HZ, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=PWM_HZ, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(20, frequency=PWM_HZ, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(21, frequency=PWM_HZ, initial_value=0, pin_factory=factory)

RIGHT_SCALE = 0.51
LEFT_SCALE = 0.50


def motor_stop():
    RIGHT_RPWM.value = 0.0
    RIGHT_LPWM.value = 0.0
    LEFT_RPWM.value = 0.0
    LEFT_LPWM.value = 0.0


def right_forward():
    RIGHT_RPWM.value = RIGHT_SCALE
    RIGHT_LPWM.value = 0.0


def right_reverse():
    RIGHT_RPWM.value = 0.0
    RIGHT_LPWM.value = RIGHT_SCALE


def left_forward():
    LEFT_RPWM.value = LEFT_SCALE
    LEFT_LPWM.value = 0.0


def left_reverse():
    LEFT_RPWM.value = 0.0
    LEFT_LPWM.value = LEFT_SCALE


# ============================================================
# STEPPER MOTOR - עבודה ישירה עם pigpio למניעת התנגשות
# ============================================================
class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("[ERROR] Stepper could not connect to pigpiod!")

        self.seq = [[1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]]
        for pin in self.pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

    def move(self, steps, direction=1, speed=0.005):
        for _ in range(int(steps)):
            for step in range(len(self.seq)):
                idx = step if direction == 1 else (len(self.seq) - 1 - step)
                pattern = self.seq[idx]
                for i, pin in enumerate(self.pins):
                    self.pi.write(pin, pattern[i])
                time.sleep(speed)
        self.stop()

    def stop(self):
        for pin in self.pins:
            self.pi.write(pin, 0)


_stepper = StepperMotor([23, 22, 27, 17])
STEPPER_STEP_CHUNK = 50


# ============================================================
# CONTROL LOOP
# ============================================================
def run_keyboard_loop():
    print("\n[READY] w/s/a/d=Drive, x=Stop, i/k=Servo, u/j=Stepper, q=Quit")
    while True:
        try:
            cmd = input("cmd> ").strip().lower()
            if not cmd: continue
            c = cmd[0]

            if c == 'q':
                break

            elif c == 'x':
                motor_stop()
                _stepper.stop()
                print("[STOP] All motors stopped")

            elif c == 'w':  # Forward
                right_forward();
                left_forward()
                print("[DRIVE] Forward")

            elif c == 's':  # Reverse
                right_reverse();
                left_reverse()
                print("[DRIVE] Reverse")

            elif c == 'a':  # Turn Left (In-place)
                right_forward();
                left_reverse()
                print("[TURN] Left")

            elif c == 'd':  # Turn Right (In-place)
                left_forward();
                right_reverse()
                print("[TURN] Right")

            elif c == 'i':
                servo_spin(1)
            elif c == 'k':
                servo_spin(0)
            elif c == 'u':
                _stepper.move(STEPPER_STEP_CHUNK, 1)
            elif c == 'j':
                _stepper.move(STEPPER_STEP_CHUNK, -1)

        except UnicodeDecodeError:
            print("[ERROR] Switch keyboard to ENGLISH!")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[ERROR] {e}")


def main():
    servo_init()
    try:
        run_keyboard_loop()
    finally:
        motor_stop()
        _stepper.stop()
        servo_cleanup()
        print("\n[CLEANUP] System stopped safely.")


if __name__ == "__main__":
    main()