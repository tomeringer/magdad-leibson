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
# ULTRASONIC SENSOR (HC-SR04 style)
#   TRIG = BCM 5 (physical 29)
#   ECHO = BCM 6 (physical 31)
# ============================================================
TRIG = 5
ECHO = 6
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ULTRA_ENABLED = True
ULTRA_STOP_CM = 40.0

ULTRA_MEASURE_PERIOD_SEC = 0.10   # measure at 10 Hz
ULTRA_PRINT_PERIOD_SEC = 0.50     # print at 2 Hz

_last_ultra_t = 0.0
_last_ultra_print_t = 0.0
_last_distance_cm: Optional[float] = None

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
    """
    direction_bit: 1 or 0 (chooses pulsewidth above/below neutral)
    """
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
#
# ACTUAL WIRING:
#   RIGHT wheel: physical 3 + 5  -> BCM 2 + 3
#   LEFT  wheel: physical 38+40  -> BCM 20+21
# ============================================================
PWM_HZ = 1000

# RIGHT wheel motor
RIGHT_RPWM = PWMOutputDevice(2, frequency=PWM_HZ, initial_value=0)
RIGHT_LPWM = PWMOutputDevice(3, frequency=PWM_HZ, initial_value=0)

# LEFT wheel motor
LEFT_RPWM = PWMOutputDevice(20, frequency=PWM_HZ, initial_value=0)
LEFT_LPWM = PWMOutputDevice(21, frequency=PWM_HZ, initial_value=0)

# Tune these if you want drift compensation
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


# Backwards-compatible names used by the rest of the code
def motor1_forward(speed: float = 1.0):  # motor1 = RIGHT wheel
    right_forward(speed)


def motor1_reverse(speed: float = 1.0):
    right_reverse(speed)


def motor2_forward(speed: float = 1.0):  # motor2 = LEFT wheel
    left_forward(speed)


def motor2_reverse(speed: float = 1.0):
    left_reverse(speed)


# ============================================================
# STEPPER (L298N 4-wire, your working implementation)
#
# ACTUAL WIRING (physical):
#   IN1=16, IN2=15, IN3=13, IN4=11
#   => BCM: 23, 22, 27, 17
# ============================================================
class StepperMotor:
    def __init__(self, name, pins):
        self.name = name
        self.pins = pins

        # Full-step sequence (4 states)
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],  # slight variant sometimes helps; keep if your wiring likes it
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


# Used already: 2,3,5,6,12,20,21
STEPPER_PINS = [23, 22, 27, 17]  # IN1..IN4
_stepper = StepperMotor("Stepper(L298N)", STEPPER_PINS)

STEPPER_SPEED_SEC = 0.005
STEPPER_STEP_CHUNK = 50


def stepper_move(steps: int, direction: int):
    """
    direction: 1 = forward, 0 = reverse
    steps: number of step-cycles
    """
    dir_pm = 1 if direction else -1
    _stepper.move(int(abs(steps)), direction=dir_pm, speed=STEPPER_SPEED_SEC)


# ============================================================
# ULTRASONIC
# ============================================================
def measure_distance_cm() -> Optional[float]:
    """
    Returns distance in cm or None on timeout.
    """
    GPIO.output(TRIG, False)
    time.sleep(0.0002)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > 0.03:
            return None

    pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - pulse_start > 0.03:
            return None

    pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150.0


def ultrasonic_tick() -> Optional[float]:
    """
    Periodically measures + periodically prints.
    Updates _last_distance_cm and returns it.
    """
    global _last_ultra_t, _last_ultra_print_t, _last_distance_cm

    now = time.time()
    if not ULTRA_ENABLED:
        return _last_distance_cm

    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        _last_distance_cm = measure_distance_cm()

    if now - _last_ultra_print_t >= ULTRA_PRINT_PERIOD_SEC:
        _last_ultra_print_t = now
        if _last_distance_cm is None:
            print("[ULTRA] dist=None (timeout)")
        else:
            print(f"[ULTRA] dist={_last_distance_cm:.1f} cm")

    return _last_distance_cm


def ultrasonic_too_close() -> bool:
    return ULTRA_ENABLED and (_last_distance_cm is not None) and (_last_distance_cm < ULTRA_STOP_CM)


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
    """
    Receives one UDP datagram (or times out).
    Expects: 0xAA <payload> 0x55
    Returns payload (0..255) or None.
    """
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
_last_ultra_block_print = 0.0

def handle_payload(payload: int):
    global _last_ultra_block_print

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    print(f"[GLOVE] payload=0x{payload:02X}")

    # FLEX bits
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # Servo (independent)
    servo_spin(f1) if f0 else servo_stop()

    # Stepper (independent)
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

    # Ultrasonic safety
    if ultrasonic_too_close() and desired in ("forward", "turn_left", "turn_right"):
        motor_stop()
        now = time.time()
        if now - _last_ultra_block_print > 0.5:
            _last_ultra_block_print = now
            print(f"[ULTRA] BLOCK DRIVE: {_last_distance_cm:.1f} cm < {ULTRA_STOP_CM:.1f} cm")
        return

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
# MAIN GLOVE LOOP (UDP PUSH-BASED)
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
            ultrasonic_tick()

            payload = read_one_udp_payload(sock)

            if payload is not None:
                now = time.time()
                gap = now - last_rx
                if gap > 0.2:
                    print(f"[GLOVE] gap {gap:.3f}s (time since last payload)")
                last_rx = now

                t0 = time.time()
                handle_payload(payload)
                t1 = time.time()
                handle_ms = (t1 - t0) * 1000.0
                if handle_ms > 50.0:
                    print(f"[DBG][CPU] handle_payload took {handle_ms:.1f}ms")

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
    """
    Controls:
      q                quit
      x                stop all (DC + servo + stepper off)
      w / s            both DC motors forward / reverse
      a / d            rotate in place left / right

      i / k            servo spin (i=one dir, k=other dir)
      o                servo stop (neutral)

      u / j            stepper chunk (u=forward, j=reverse)
      U / J            stepper bigger chunk (5x)
    """
    print("[KEYBOARD] Control mode")
    print("Commands: w/s/a/d (drive), x (stop), i/k (servo), o (servo stop), u/j (stepper), q (quit)")

    try:
        while True:
            ultrasonic_tick()

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
                if ultrasonic_too_close():
                    motor_stop()
                    print(f"[ULTRA] BLOCK FORWARD: {_last_distance_cm:.1f} cm < {ULTRA_STOP_CM:.1f} cm")
                else:
                    motor1_forward()
                    motor2_forward()
                    print("[KEYBOARD] forward")

            elif c == "s":
                motor1_reverse()
                motor2_reverse()
                print("[KEYBOARD] reverse")

            elif c == "a":
                if ultrasonic_too_close():
                    motor_stop()
                    print(f"[ULTRA] BLOCK TURN: {_last_distance_cm:.1f} cm < {ULTRA_STOP_CM:.1f} cm")
                else:
                    motor1_reverse()
                    motor2_forward()
                    print("[KEYBOARD] turn left (in place)")

            elif c == "d":
                if ultrasonic_too_close():
                    motor_stop()
                    print(f"[ULTRA] BLOCK TURN: {_last_distance_cm:.1f} cm < {ULTRA_STOP_CM:.1f} cm")
                else:
                    motor1_forward()
                    motor2_reverse()
                    print("[KEYBOARD] turn right (in place)")

            # Servo
            elif c == "i":
                servo_spin(1)
                print("[KEYBOARD] servo spin dir=1")

            elif c == "k":
                servo_spin(0)
                print("[KEYBOARD] servo spin dir=0")

            elif c == "o":
                servo_stop()
                print("[KEYBOARD] servo stop")

            # Stepper
            elif c == "u":
                stepper_move(STEPPER_STEP_CHUNK, 1)
                print(f"[KEYBOARD] stepper forward {STEPPER_STEP_CHUNK}")

            elif c == "j":
                stepper_move(STEPPER_STEP_CHUNK, 0)
                print(f"[KEYBOARD] stepper reverse {STEPPER_STEP_CHUNK}")

            elif c == "U":
                stepper_move(STEPPER_STEP_CHUNK * 5, 1)
                print(f"[KEYBOARD] stepper forward {STEPPER_STEP_CHUNK * 5}")

            elif c == "J":
                stepper_move(STEPPER_STEP_CHUNK * 5, 0)
                print(f"[KEYBOARD] stepper reverse {STEPPER_STEP_CHUNK * 5}")

            else:
                print("[KEYBOARD] unknown command")

    except KeyboardInterrupt:
        pass
    finally:
        motor_stop()
        servo_stop()
        _stepper.stop()
        print("[KEYBOARD] exiting, stopped motors/servo/stepper")


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
