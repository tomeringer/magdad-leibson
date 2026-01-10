# -*- coding: utf-8 -*-

from gpiozero import OutputDevice
import serial
import time
import RPi.GPIO as GPIO
import os

# ============================================================
# INPUT MODE
# ============================================================
MODE = "KEYBOARD"   # "GLOVE" or "KEYBOARD"

# ============================================================
# BLUETOOTH / SERIAL CONFIG
# ============================================================
RFCOMM_PORT = "/dev/rfcomm0"
GLOVE_BAUD = 9600

BT_FRAME_START = 0xAA          # Arduino -> Pi
BT_FRAME_END   = 0x55

REQUEST_TIMEOUT_SEC = 0.4
CONTROL_PERIOD_SEC  = 0.01
SILENCE_STOP_SEC    = 0.7

# ============================================================
# GPIO SETUP (BCM numbering)
# ============================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================
# YOUR CURRENT PIN MAP (from your chart)
# ============================================================

# --- DC motors (2 pins each; simple direction control) ---
DC1_A = 2    # physical 3
DC1_B = 3    # physical 5
DC2_A = 20   # physical 38
DC2_B = 21   # physical 40

# --- Servo signal ---
SERVO_PIN = 18  # physical 12

# --- Stepper driver = L298N style (IN1..IN4) ---
# You wired: IN1->phys16(GPIO23), IN2->phys15(GPIO22), IN3->phys13(GPIO27), IN4->phys11(GPIO17)
STEP_IN1 = 23   # physical 16
STEP_IN2 = 22   # physical 15
STEP_IN3 = 27   # physical 13
STEP_IN4 = 17   # physical 11

# --- Ultrasonic sensors (two of them) ---
US1_TRIG = 5     # physical 29
US1_ECHO = 6     # physical 31

US2_TRIG = 13    # physical 33
US2_ECHO = 19    # physical 35

THRESHOLD_CM = 50

# Setup ultrasonic GPIOs
for trig in (US1_TRIG, US2_TRIG):
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, False)

for echo in (US1_ECHO, US2_ECHO):
    GPIO.setup(echo, GPIO.IN)

# ============================================================
# SERVO (pigpio)
# ============================================================
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
        print("WARNING: pigpio daemon not running (run: sudo pigpiod)")
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
# DC MOTORS (2-pin control per motor)
# ============================================================
M1_A = OutputDevice(DC1_A, active_high=True, initial_value=False)
M1_B = OutputDevice(DC1_B, active_high=True, initial_value=False)

M2_A = OutputDevice(DC2_A, active_high=True, initial_value=False)
M2_B = OutputDevice(DC2_B, active_high=True, initial_value=False)


def motor_stop():
    M1_A.off(); M1_B.off()
    M2_A.off(); M2_B.off()


def motor1_forward():  M1_A.on();  M1_B.off()
def motor1_reverse():  M1_A.off(); M1_B.on()
def motor2_forward():  M2_A.on();  M2_B.off()
def motor2_reverse():  M2_A.off(); M2_B.on()


# ============================================================
# STEPPER (L298N IN1..IN4)
# ============================================================
IN1 = OutputDevice(STEP_IN1, active_high=True, initial_value=False)
IN2 = OutputDevice(STEP_IN2, active_high=True, initial_value=False)
IN3 = OutputDevice(STEP_IN3, active_high=True, initial_value=False)
IN4 = OutputDevice(STEP_IN4, active_high=True, initial_value=False)

STEPPER_STEP_DELAY = 0.002   # L298N is slower than A4988; 0.001..0.005 typical
STEPPER_STEP_CHUNK = 20

# Half-step sequence (8 states) for a 4-wire bipolar stepper via H-bridges
# If direction is wrong or it "buzzes", we can reverse this or swap coil order.
HALF_STEP_SEQ = [
    (1, 0, 0, 0),
    (1, 0, 1, 0),
    (0, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 0),
    (0, 1, 0, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
]


def _stepper_set(a, b, c, d):
    IN1.value = 1 if a else 0
    IN2.value = 1 if b else 0
    IN3.value = 1 if c else 0
    IN4.value = 1 if d else 0


def stepper_release():
    _stepper_set(0, 0, 0, 0)


def stepper_move(steps: int, direction: int):
    """
    steps: number of half-steps to execute
    direction: 1 forward, 0 reverse
    """
    seq = HALF_STEP_SEQ if direction else list(reversed(HALF_STEP_SEQ))
    for i in range(abs(steps)):
        a, b, c, d = seq[i % len(seq)]
        _stepper_set(a, b, c, d)
        time.sleep(STEPPER_STEP_DELAY)
    stepper_release()


# ============================================================
# ULTRASONIC (two sensors)
# ============================================================
def measure_distance(trig_pin: int, echo_pin: int):
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    t0 = time.time()
    while GPIO.input(echo_pin) == 0:
        if time.time() - t0 > 0.05:
            return None

    pulse_start = time.time()
    while GPIO.input(echo_pin) == 1:
        if time.time() - pulse_start > 0.05:
            return None

    pulse_end = time.time()
    return round((pulse_end - pulse_start) * 17150, 2)


def measure_both_ultrasonics():
    d1 = measure_distance(US1_TRIG, US1_ECHO)
    d2 = measure_distance(US2_TRIG, US2_ECHO)
    return d1, d2


# ============================================================
# SERIAL HELPERS
# ============================================================
def open_rfcomm_serial_forever(port, baud):
    while True:
        try:
            if not os.path.exists(port):
                time.sleep(0.5)
                continue
            ser = serial.Serial(port, baud, timeout=0.05)
            ser.reset_input_buffer()
            return ser
        except Exception as e:
            print("[BT] Waiting for rfcomm:", e)
            time.sleep(1)


def read_latest_frame_nonblocking(ser, buf: bytearray):
    try:
        n_wait = int(getattr(ser, "in_waiting", 0) or 0)
    except Exception:
        n_wait = 0

    to_read = 512 if n_wait <= 0 else min(n_wait, 2048)
    chunk = ser.read(to_read)

    if chunk:
        buf.extend(chunk)

    MAX_BUF = 256
    if len(buf) > MAX_BUF:
        del buf[:-MAX_BUF]

    i = len(buf) - 3
    while i >= 0:
        if buf[i] == BT_FRAME_START and buf[i + 2] == BT_FRAME_END:
            payload = buf[i + 1]
            del buf[:i + 3]
            return payload
        i -= 1

    if len(buf) > 2:
        del buf[:-2]
    return None


# ============================================================
# PAYLOAD HANDLER
# ============================================================
def handle_payload(payload: int):
    flex = (payload >> 4) & 0x0F
    roll_code  = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    print(f"[GLOVE] payload=0x{payload:02X}")

    # FLEX bits
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

    # Ultrasonics print
    d1, d2 = measure_both_ultrasonics()
    print(f"US1: {d1} cm | US2: {d2} cm")

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
# MAIN GLOVE LOOP
# ============================================================
def run_glove_loop():
    print("[GLOVE] Bluetooth listen mode")
    ser = None
    rx_buf = bytearray()

    last_rx = time.time()
    last_stop_action = 0.0

    try:
        while True:
            if ser is None:
                ser = open_rfcomm_serial_forever(RFCOMM_PORT, GLOVE_BAUD)
                rx_buf.clear()
                last_rx = time.time()
                last_stop_action = 0.0

            payload = read_latest_frame_nonblocking(ser, rx_buf)

            if payload is not None:
                last_rx = time.time()
                handle_payload(payload)
            else:
                now = time.time()
                if (now - last_rx) > SILENCE_STOP_SEC and (now - last_stop_action) > 0.5:
                    motor_stop()
                    servo_stop()
                    stepper_release()
                    last_stop_action = now

            time.sleep(CONTROL_PERIOD_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.close()


def run_keyboard_loop():
    print("[KEYBOARD] Control mode")
    print("Commands: w/s/a/d (drive), x (stop), i/k (servo), o (servo stop), u/j (stepper), p (print ultrasonics), q (quit)")

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
                stepper_release()
                print("[KEYBOARD] stop all")

            # DC motors
            elif c == "w":
                motor1_forward(); motor2_forward()
                print("[KEYBOARD] forward")

            elif c == "s":
                motor1_reverse(); motor2_reverse()
                print("[KEYBOARD] reverse")

            elif c == "a":
                motor1_reverse(); motor2_forward()
                print("[KEYBOARD] turn left (in place)")

            elif c == "d":
                motor1_forward(); motor2_reverse()
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

            # Ultrasonics
            elif c == "p":
                d1, d2 = measure_both_ultrasonics()
                print(f"[KEYBOARD] US1: {d1} cm | US2: {d2} cm")

            else:
                print("[KEYBOARD] unknown command")

    except KeyboardInterrupt:
        pass
    finally:
        motor_stop()
        servo_stop()
        stepper_release()
        print("[KEYBOARD] exiting, stopped outputs")


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
        servo_cleanup()
        stepper_release()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
