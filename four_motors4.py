#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gpiozero import PWMOutputDevice
import time
import socket
from typing import Optional
import RPi.GPIO as GPIO

# ============================================================
# INPUT MODE
# ============================================================
MODE = "KEYBOARD"   # "GLOVE" or "KEYBOARD"

# ============================================================
# WIFI / UDP GLOVE CONFIG
# ============================================================
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210

FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

# ============================================================
# GPIO SETUP
# ============================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================
# MOTION STATE (for ultrasonic interrupt)
# ============================================================
_current_motion = "stop"   # stop | forward | reverse | turn_left | turn_right

# ============================================================
# ULTRASONIC SENSORS
# ============================================================
US1_TRIG = 5
US1_ECHO = 6
US2_TRIG = 13
US2_ECHO = 19

US_STOP_DISTANCE_CM = 50.0
US_TIMEOUT_SEC = 0.03

ULTRA_MEASURE_PERIOD_SEC = 0.10
ULTRA_PRINT_PERIOD_SEC = 0.50

GPIO.setup(US1_TRIG, GPIO.OUT)
GPIO.setup(US1_ECHO, GPIO.IN)
GPIO.setup(US2_TRIG, GPIO.OUT)
GPIO.setup(US2_ECHO, GPIO.IN)

GPIO.output(US1_TRIG, GPIO.LOW)
GPIO.output(US2_TRIG, GPIO.LOW)

_last_ultra_t = 0.0
_last_ultra_print_t = 0.0
_last_distances = [None, None]

def _measure_distance_cm(trig, echo) -> Optional[float]:
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    t0 = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - t0 > US_TIMEOUT_SEC:
            return None

    pulse_start = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - pulse_start > US_TIMEOUT_SEC:
            return None

    pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150.0

def ultrasonic_tick():
    global _last_ultra_t, _last_ultra_print_t, _last_distances

    now = time.time()

    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        _last_distances = [
            _measure_distance_cm(US1_TRIG, US1_ECHO),
            _measure_distance_cm(US2_TRIG, US2_ECHO),
        ]

    if now - _last_ultra_print_t >= ULTRA_PRINT_PERIOD_SEC:
        _last_ultra_print_t = now
        d1, d2 = _last_distances
        print(f"[ULTRA] d1={None if d1 is None else f'{d1:.1f}'} cm | "
              f"d2={None if d2 is None else f'{d2:.1f}'} cm")

def obstacle_too_close() -> bool:
    for d in _last_distances:
        if d is not None and d < US_STOP_DISTANCE_CM:
            return True
    return False

def ultrasonic_interrupt_check():
    global _current_motion

    if _current_motion in ("forward", "turn_left", "turn_right"):
        if obstacle_too_close():
            motor_stop()
            _current_motion = "stop"
            print(f"[ULTRA][INTERRUPT] Emergency stop (<{US_STOP_DISTANCE_CM} cm)")

# ============================================================
# SERVO (pigpio)
# ============================================================
SERVO_PIN = 12
SERVO_NEUTRAL_US = 1500
SERVO_SPIN_DELTA_US = 200

try:
    import pigpio
except ImportError:
    pigpio = None

_pi = None

def servo_init():
    global _pi
    if pigpio is None:
        print("[SERVO] pigpio not installed")
        return
    _pi = pigpio.pi()
    if not _pi.connected:
        print("[SERVO] pigpio daemon not running")
        _pi = None
        return
    _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)
    print("[SERVO] initialized")

def servo_stop():
    if _pi:
        _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)

def servo_spin(direction_bit: int):
    if _pi is None:
        return
    pw = SERVO_NEUTRAL_US + SERVO_SPIN_DELTA_US if direction_bit else SERVO_NEUTRAL_US - SERVO_SPIN_DELTA_US
    _pi.set_servo_pulsewidth(SERVO_PIN, pw)

def servo_cleanup():
    global _pi
    if _pi:
        servo_stop()
        _pi.stop()
        _pi = None

# ============================================================
# DC MOTORS (IBT-2)
# ============================================================
PWM_HZ = 1000

RIGHT_RPWM = PWMOutputDevice(2, frequency=PWM_HZ, initial_value=0)
RIGHT_LPWM = PWMOutputDevice(3, frequency=PWM_HZ, initial_value=0)
LEFT_RPWM  = PWMOutputDevice(20, frequency=PWM_HZ, initial_value=0)
LEFT_LPWM  = PWMOutputDevice(21, frequency=PWM_HZ, initial_value=0)

RIGHT_SCALE = 0.51
LEFT_SCALE = 0.50

def _clamp01(x): return max(0.0, min(1.0, x))

def motor_stop():
    RIGHT_RPWM.value = 0
    RIGHT_LPWM.value = 0
    LEFT_RPWM.value = 0
    LEFT_LPWM.value = 0

def motor1_forward():
    RIGHT_RPWM.value = RIGHT_SCALE
    RIGHT_LPWM.value = 0

def motor1_reverse():
    RIGHT_RPWM.value = 0
    RIGHT_LPWM.value = RIGHT_SCALE

def motor2_forward():
    LEFT_RPWM.value = LEFT_SCALE
    LEFT_LPWM.value = 0

def motor2_reverse():
    LEFT_RPWM.value = 0
    LEFT_LPWM.value = LEFT_SCALE

# ============================================================
# STEPPER (L298N)
# ============================================================
class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.seq = [
            [1,0,1,0],
            [0,1,1,0],
            [0,1,0,1],
            [0,0,1,1],
        ]
        for p in pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

    def move(self, steps, direction, speed):
        try:
            for _ in range(steps):
                for i in range(4):
                    idx = i if direction == 1 else 3 - i
                    for p, v in zip(self.pins, self.seq[idx]):
                        GPIO.output(p, v)
                    time.sleep(speed)
        finally:
            self.stop()

    def stop(self):
        for p in self.pins:
            GPIO.output(p, 0)

STEPPER_PINS = [23, 22, 27, 17]
_stepper = StepperMotor(STEPPER_PINS)

STEPPER_STEP_CHUNK = 50
STEPPER_SPEED_SEC = 0.005

def stepper_move(steps, direction):
    _stepper.move(abs(steps), 1 if direction else -1, STEPPER_SPEED_SEC)

# ============================================================
# UDP HELPERS
# ============================================================
def open_udp_socket_forever(ip, port):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((ip, port))
            s.settimeout(0.05)
            print(f"[UDP] Listening on {ip}:{port}")
            return s
        except Exception:
            time.sleep(1)

def read_one_udp_payload(sock):
    try:
        data, _ = sock.recvfrom(1024)
    except socket.timeout:
        return None
    if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
        return data[1]
    return None

# ============================================================
# PAYLOAD HANDLER
# ============================================================
def handle_payload(payload: int):
    global _current_motion

    print(f"[GLOVE] payload=0x{payload:02X}")

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    servo_spin(f1) if f0 else servo_stop()

    if f2 and not f3:
        stepper_move(STEPPER_STEP_CHUNK, 1)
    elif f3 and not f2:
        stepper_move(STEPPER_STEP_CHUNK, 0)

    desired = "stop"
    if pitch_code == 1:
        desired = "forward"
    elif pitch_code == 2:
        desired = "reverse"
    elif roll_code == 1:
        desired = "turn_right"
    elif roll_code == 2:
        desired = "turn_left"

    if desired in ("forward", "turn_left", "turn_right") and obstacle_too_close():
        motor_stop()
        _current_motion = "stop"
        print("[ULTRA] BLOCK command")
        return

    if desired == "forward":
        motor1_forward(); motor2_forward()
    elif desired == "reverse":
        motor1_reverse(); motor2_reverse()
    elif desired == "turn_right":
        motor1_forward(); motor2_reverse()
    elif desired == "turn_left":
        motor1_reverse(); motor2_forward()
    else:
        motor_stop()

    _current_motion = desired

# ============================================================
# MAIN LOOPS
# ============================================================
def run_glove_loop():
    sock = open_udp_socket_forever(UDP_LISTEN_IP, UDP_PORT)
    last_rx = time.time()

    try:
        while True:
            ultrasonic_tick()
            ultrasonic_interrupt_check()

            payload = read_one_udp_payload(sock)
            if payload is not None:
                last_rx = time.time()
                handle_payload(payload)
            elif time.time() - last_rx > SILENCE_STOP_SEC:
                motor_stop()
                servo_stop()
                _stepper.stop()
                _current_motion = "stop"

            time.sleep(CONTROL_PERIOD_SEC)
    finally:
        sock.close()

def run_keyboard_loop():
    global _current_motion
    print("[KEYBOARD] mode")

    try:
        while True:
            ultrasonic_tick()
            ultrasonic_interrupt_check()

            c = input("cmd> ").strip()[:1]
            if c == "q":
                break
            elif c == "w":
                if not obstacle_too_close():
                    motor1_forward(); motor2_forward()
                    _current_motion = "forward"
            elif c == "s":
                motor1_reverse(); motor2_reverse()
                _current_motion = "reverse"
            elif c == "a":
                if not obstacle_too_close():
                    motor1_reverse(); motor2_forward()
                    _current_motion = "turn_left"
            elif c == "d":
                if not obstacle_too_close():
                    motor1_forward(); motor2_reverse()
                    _current_motion = "turn_right"
            elif c == "x":
                motor_stop(); servo_stop(); _stepper.stop()
                _current_motion = "stop"
            elif c == "u":
                stepper_move(STEPPER_STEP_CHUNK, 1)
            elif c == "j":
                stepper_move(STEPPER_STEP_CHUNK, 0)
    finally:
        motor_stop()
        servo_stop()
        _stepper.stop()

# ============================================================
# MAIN
# ============================================================
def main():
    servo_init()
    try:
        if MODE.upper() == "GLOVE":
            run_glove_loop()
        else:
            run_keyboard_loop()
    finally:
        motor_stop()
        _stepper.stop()
        servo_cleanup()
        GPIO.cleanup()
        print("[EXIT] clean shutdown")

if __name__ == "__main__":
    main()
