#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gpiozero import OutputDevice
import time
import RPi.GPIO as GPIO
import socket


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
FRAME_END   = 0x55

CONTROL_PERIOD_SEC  = 0.01
SILENCE_STOP_SEC    = 0.7

# ============================================================
# GPIO SETUP (BCM numbering)
# ============================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ultrasonic sensor
TRIG = 5
ECHO = 6
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
THRESHOLD_CM = 50

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
        print("WARNING: pigpio daemon not running")
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
# DC MOTORS (IBT-2)
# ============================================================
M1_RPWM = OutputDevice(18)
M1_LPWM = OutputDevice(23)
M2_RPWM = OutputDevice(24)
M2_LPWM = OutputDevice(25)


def motor_stop():
    M1_RPWM.off()
    M1_LPWM.off()
    M2_RPWM.off()
    M2_LPWM.off()


def motor1_forward():  M1_RPWM.on();  M1_LPWM.off()
def motor1_reverse():  M1_RPWM.off(); M1_LPWM.on()
def motor2_forward():  M2_RPWM.on();  M2_LPWM.off()
def motor2_reverse():  M2_RPWM.off(); M2_LPWM.on()


# ============================================================
# STEPPER (L298N 4-wire, your working implementation)
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
            [1, 0, 0, 1],
        ]

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

    def move(self, steps, direction=1, speed=0.005):
        """
        Rotate the motor.
        steps: number of step-cycles to run
        direction: 1 (one way), -1 (the other way)
        speed: delay between micro-states in seconds (smaller=faster)
        """
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")

        try:
            for _ in range(steps):
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


# Used already: 5,6,12,18,23,24,25
STEPPER_PINS = [16, 19, 20, 21]   # L298N IN1..IN4
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
def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - start > 0.05:
            return None

    pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - pulse_start > 0.05:
            return None

    pulse_end = time.time()
    return round((pulse_end - pulse_start) * 17150, 2)


# ============================================================
# UDP HELPERS
# ============================================================
def open_udp_socket_forever(bind_ip: str, bind_port: int):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((bind_ip, bind_port))
            s.settimeout(0.2)  # IMPORTANT: blocking with timeout (like your working script)
            print(f"[UDP] Listening on {bind_ip}:{bind_port}")
            return s
        except Exception as e:
            print("[UDP] Failed to bind, retrying:", e)
            time.sleep(1.0)



def read_latest_udp_payload(sock):
    """
    Waits up to socket timeout for a UDP datagram.
    Expects exactly: 0xAA <payload> 0x55
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
        return data[1]
    return None



# ============================================================
# PAYLOAD HANDLER
# ============================================================
def handle_payload(payload: int):
    flex = (payload >> 4) & 0x0F
    roll_code  = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    print(f"[GLOVE] payload=0x{payload:02X}")

    # FLEX
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

    # DC motors
    if pitch_code == 0b01:
        motor1_forward(); motor2_forward()
    elif pitch_code == 0b10:
        motor1_reverse(); motor2_reverse()
    else:
        if roll_code == 0b01:
            motor1_forward(); motor2_reverse()
        elif roll_code == 0b10:
            motor1_reverse(); motor2_forward()
        else:
            motor_stop()


# ============================================================
# MAIN GLOVE LOOP (UDP PUSH-BASED)
# ============================================================
def run_glove_loop():
    print("[GLOVE] WiFi UDP mode (listen-only)")
    sock = None

    last_rx = time.time()
    last_stop_action = 0.0

    last_silence_report = 0.0
    SILENCE_REPORT_EVERY = 0.5

    last_loop_t = time.time()

    try:
        while True:
            if sock is None:
                sock = open_udp_socket_forever(UDP_LISTEN_IP, UDP_PORT)
                now = time.time()
                last_rx = now
                last_stop_action = 0.0
                last_silence_report = 0.0
                last_loop_t = now

            payload = read_latest_udp_payload(sock)

            if payload is not None:
                now = time.time()
                gap = now - last_rx
                if gap > 0.2:
                    print(f"[GLOVE] gap {gap:.3f}s (time since last payload)")
                last_rx = now

                t0 = time.time()
                handle_payload(int(payload))
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

            loop_end = time.time()
            loop_dt = loop_end - last_loop_t
            last_loop_t = loop_end
            if loop_dt > 0.2:
                print(f"[DBG][LOOP] loop_dt={loop_dt:.3f}s (loop stalled)")

            time.sleep(CONTROL_PERIOD_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        if sock:
            sock.close()


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

            # DC motors
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
