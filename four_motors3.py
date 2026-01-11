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
#   ESP sends 3 bytes per datagram: 0xAA <payload> 0x55
#   Pi listens on UDP_PORT on all interfaces
# ============================================================
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210

FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

# ============================================================
# GPIO SETUP (BCM numbering)
# ============================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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
        print("WARNING: pigpio not installed")
        return
    _pi = pigpio.pi()
    if not _pi.connected:
        print("WARNING: pigpio daemon not running (sudo systemctl enable --now pigpiod)")
        _pi = None
        return
    _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)


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
# DC MOTORS (IBT-2) -- PWM + trim
# ============================================================
PWM_HZ = 1000

# RIGHT wheel motor
RIGHT_RPWM = PWMOutputDevice(2, frequency=PWM_HZ, initial_value=0)
RIGHT_LPWM = PWMOutputDevice(3, frequency=PWM_HZ, initial_value=0)

# LEFT wheel motor
LEFT_RPWM = PWMOutputDevice(20, frequency=PWM_HZ, initial_value=0)
LEFT_LPWM = PWMOutputDevice(21, frequency=PWM_HZ, initial_value=0)

RIGHT_SCALE = 0.51
LEFT_SCALE = 0.50


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def motor_stop():
    RIGHT_RPWM.value = 0.0
    RIGHT_LPWM.value = 0.0
    LEFT_RPWM.value = 0.0
    LEFT_LPWM.value = 0.0


def right_forward(speed: float = 1.0):
    s = _clamp01(speed) * RIGHT_SCALE
    RIGHT_RPWM.value = s
    RIGHT_LPWM.value = 0.0


def right_reverse(speed: float = 1.0):
    s = _clamp01(speed) * RIGHT_SCALE
    RIGHT_RPWM.value = 0.0
    RIGHT_LPWM.value = s


def left_forward(speed: float = 1.0):
    s = _clamp01(speed) * LEFT_SCALE
    LEFT_RPWM.value = s
    LEFT_LPWM.value = 0.0


def left_reverse(speed: float = 1.0):
    s = _clamp01(speed) * LEFT_SCALE
    LEFT_RPWM.value = 0.0
    LEFT_LPWM.value = s


def motor1_forward(speed: float = 1.0):
    right_forward(speed)


def motor1_reverse(speed: float = 1.0):
    right_reverse(speed)


def motor2_forward(speed: float = 1.0):
    left_forward(speed)


def motor2_reverse(speed: float = 1.0):
    left_reverse(speed)


# ============================================================
# STEPPER (L298N 4-wire)
# ============================================================
class StepperMotor:
    def __init__(self, name, pins):
        self.name = name
        self.pins = pins
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
        ]

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

    def move(self, steps, direction=1, speed=0.005):
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")

        try:
            for _ in range(int(steps)):
                for step in range(len(self.seq)):
                    step_index = step if direction == 1 else (len(self.seq) - 1 - step)
                    pattern = self.seq[step_index]
                    for i, pin in enumerate(self.pins):
                        GPIO.output(pin, pattern[i])
                    time.sleep(speed)
        finally:
            self.stop()

    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, 0)


STEPPER_PINS = [23, 22, 27, 17]
_stepper = StepperMotor("Stepper(L298N)", STEPPER_PINS)

STEPPER_SPEED_SEC = 0.005
STEPPER_STEP_CHUNK = 50


def stepper_move(steps: int, direction: int):
    dir_pm = 1 if direction else -1
    _stepper.move(int(abs(steps)), direction=dir_pm, speed=STEPPER_SPEED_SEC)


# ============================================================
# UDP HELPERS
# ============================================================
def open_udp_socket_forever(bind_ip: str, bind_port: int):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((bind_ip, bind_port))
            s.settimeout(0.05)
            print(f"[UDP] Listening on {bind_ip}:{bind_port}")
            return s
        except Exception as e:
            print("[UDP] Failed to bind, retrying:", e)
            time.sleep(1.0)


def read_one_udp_payload(sock) -> Optional[int]:
    try:
        data, _addr = sock.recvfrom(1024)
    except socket.timeout:
        return None
    except Exception as e:
        print("[UDP] recv error:", e)
        return None

    if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
        return int(data[1])
    return None


# ============================================================
# PAYLOAD HANDLER
# ============================================================
def handle_payload(payload: int):
    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    print(f"[GLOVE] payload=0x{payload:02X}")

    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # Servo
    servo_spin(f1) if f0 else servo_stop()

    # Stepper
    if f2 and not f3:
        stepper_move(STEPPER_STEP_CHUNK, 1)
    elif f3 and not f2:
        stepper_move(STEPPER_STEP_CHUNK, 0)

    # Decide desired DC drive action
    desired = "stop"
    if pitch_code == 0b01:
        desired = "forward"
    elif pitch_code == 0b10:
        desired = "reverse"
    else:
        if roll_code == 0b01:
            desired = "turn_right"
        elif roll_code == 0b10:
            desired = "turn_left"
        else:
            desired = "stop"

    # Execute DC drive
    if desired == "forward":
        motor1_forward()
        motor2_forward()
    elif desired == "reverse":
        motor1_reverse()
        motor2_reverse()
    elif desired == "turn_right":
        motor1_forward()
        motor2_reverse()
    elif desired == "turn_left":
        motor1_reverse()
        motor2_forward()
    else:
        motor_stop()


# ============================================================
# MAIN GLOVE LOOP
# ============================================================
def run_glove_loop():
    print("[GLOVE] WiFi UDP mode (listen-only)")
    sock = open_udp_socket_forever(UDP_LISTEN_IP, UDP_PORT)

    last_rx = time.time()
    last_stop_action = 0.0
    last_silence_report = 0.0
    SILENCE_REPORT_EVERY = 0.5

    try:
        while True:
            payload = read_one_udp_payload(sock)

            if payload is not None:
                now = time.time()
                last_rx = now
                handle_payload(payload)
            else:
                now = time.time()
                silent_for = now - last_rx

                if silent_for > 0.3 and (now - last_silence_report) >= SILENCE_REPORT_EVERY:
                    print(f"[DBG][SILENCE] silent_for={silent_for:.2f}s")
                    last_silence_report = now

                if silent_for > SILENCE_STOP_SEC and (now - last_stop_action) > 0.5:
                    motor_stop()
                    servo_stop()
                    _stepper.stop()
                    last_stop_action = now

            time.sleep(CONTROL_PERIOD_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass


# ============================================================
# KEYBOARD MODE
# ============================================================
def run_keyboard_loop():
    print("[KEYBOARD] Control mode")
    print("Commands: w/s/a/d (drive), x (stop), i/k (servo), o (servo stop), u/j (stepper), q (quit)")

    try:
        while True:
            cmd = input("cmd> ").strip()
            if not cmd:
                continue

            c = cmd[0]

            if c in ("q", "Q"):
                break

            elif c == "x":
                motor_stop()
                servo_stop()
                _stepper.stop()
                print("[KEYBOARD] stop all")

            elif c == "w":
                motor1_forward()
                motor2_forward()
                print("[KEYBOARD] forward")

            elif c == "s":
                motor1_reverse()
                motor2_reverse()
                print("[KEYBOARD] reverse")

            elif c == "a":
                motor1_reverse()
                motor2_forward()
                print("[KEYBOARD] turn left (in place)")

            elif c == "d":
                motor1_forward()
                motor2_reverse()
                print("[KEYBOARD] turn right (in place)")

            elif c == "i":
                servo_spin(1)
            elif c == "k":
                servo_spin(0)
            elif c == "o":
                servo_stop()
            elif c == "u":
                stepper_move(STEPPER_STEP_CHUNK, 1)
            elif c == "j":
                stepper_move(STEPPER_STEP_CHUNK, 0)
            elif c == "U":
                stepper_move(STEPPER_STEP_CHUNK * 5, 1)
            elif c == "J":
                stepper_move(STEPPER_STEP_CHUNK * 5, 0)
            else:
                print("[KEYBOARD] unknown command")

    except KeyboardInterrupt:
        pass
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


if __name__ == "__main__":
    main()