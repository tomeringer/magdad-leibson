# -*- coding: utf-8 -*-
from gpiozero import OutputDevice
import serial
import time
import RPi.GPIO as GPIO  # for ultrasonic distance sensor
import os

# ===== INPUT MODE =====
# "GLOVE"   -> read packets from /dev/rfcomm0 (HC-05)
# "KEYBOARD" -> read commands from keyboard (stdin)
MODE = "GLOVE"  # change to "GLOVE" to use glove input

# ===== SERVO (signal) GPIO =====
# Use GPIO 12 (BCM 12) as the servo signal pin (free + PWM-capable).
SERVO_PIN = 12

# Servo control uses pigpio for stable PWM
try:
    import pigpio
except ImportError:
    pigpio = None


# ============================================================
# HARDWARE MAPPING (BCM numbering, Raspberry Pi)
#
#  IBT1 (Motor 1):
#    RPWM  -> GPIO 18
#    LPWM  -> GPIO 23
#
#  IBT2 (Motor 2):
#    RPWM  -> GPIO 24
#    LPWM  -> GPIO 25
#
#  STEPPER DRIVER (A4988-style):
#    STEP -> GPIO 17
#    DIR  -> GPIO 27
#
#  ULTRASONIC SENSOR (HC-SR04 style):
#    TRIG -> GPIO 5
#    ECHO -> GPIO 6 (via 5V -> 3.3V voltage divider)
#
# ============================================================
# INPUT FORMAT (from glove, CURRENT)
#
#  Arduino sends frames:
#      START (0xAA),
#      payload (1 byte),
#      END (0x55)
#
#  payload byte layout:
#    bits 7..4 = flex bits (4 sensors)
#    bits 3..2 = roll code  (2-bit: 00 neutral, 01 +th, 10 -th)
#    bits 1..0 = pitch code (2-bit: 00 neutral, 01 +th, 10 -th)
#
#  NOTE: Arduino "9-bit" print is debug only; transmitted payload is 8 bits.
# ============================================================

START = 0xAA
END = 0x55

# --------- DISTANCE SENSOR SETUP (RPi.GPIO) ---------
TRIG = 5
ECHO = 6
THRESHOLD_CM = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

RFCOMM_PORT = "/dev/rfcomm0"
GLOVE_BAUD = 9600  # must match Arduino Serial1.begin(...)


def open_rfcomm_serial_forever(port: str, baud: int) -> serial.Serial:
    """
    Keeps trying until /dev/rfcomm0 exists and can be opened.
    Does NOT do any bluetooth connecting (systemd rfcomm service does that).
    """
    while True:
        try:
            if not os.path.exists(port):
                time.sleep(0.5)
                continue

            ser = serial.Serial(port, baud, timeout=1)
            ser.reset_input_buffer()
            return ser

        except (serial.SerialException, OSError) as e:
            print(f"[GLOVE] Waiting for serial {port}... ({e})")
            time.sleep(1)


def measure_distance():
    """
    Returns distance in cm, or None if no echo/invalid.
    Uses a single ping; safe against timeouts.
    """
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = None
    pulse_end = None

    start_deadline = time.time() + 0.05
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > start_deadline:
            return None

    end_deadline = time.time() + 0.05
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > end_deadline:
            return None

    if pulse_start is None or pulse_end is None:
        return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)


# --------- SERVO SETUP (pigpio) ---------
_pi = None

# For a continuous-rotation servo:
SERVO_NEUTRAL_US = 1500
SERVO_SPIN_DELTA_US = 200  # adjust if needed (150..300 common)


def servo_init():
    global _pi
    if pigpio is None:
        print("WARNING: pigpio not installed; servo disabled.")
        _pi = None
        return

    _pi = pigpio.pi()
    if not _pi.connected:
        print("WARNING: pigpio daemon not running; servo disabled.")
        _pi = None
        return

    _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)


def servo_stop():
    if _pi is None:
        return
    _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)


def servo_spin(direction_bit: int):
    """
    direction_bit: 0/1 -> selects which side of neutral pulse to use.
    """
    if _pi is None:
        return

pw = SERVO_NEUTRAL_US - SERVO_SPIN_DELTA_US if direction_bit == 0 else SERVO_NEUTRAL_US + SERVO_SPIN_DELTA_US
_pi.set_servo_pulsewidth(SERVO_PIN, pw)


def servo_cleanup():
    global _pi
    if _pi is None:
        return
    servo_stop()
    _pi.stop()
    _pi = None


# --------- DC MOTOR SETUP (gpiozero) ---------
M1_RPWM = OutputDevice(18, active_high=True, initial_value=False)
M1_LPWM = OutputDevice(23, active_high=True, initial_value=False)

M2_RPWM = OutputDevice(24, active_high=True, initial_value=False)
M2_LPWM = OutputDevice(25, active_high=True, initial_value=False)


def motor1_stop():
    M1_RPWM.off()
    M1_LPWM.off()


def motor1_forward():
    M1_RPWM.on()
    M1_LPWM.off()


def motor1_reverse():
    M1_RPWM.off()
    M1_LPWM.on()


def motor2_stop():
    M2_RPWM.off()
    M2_LPWM.off()


def motor2_forward():
    M2_RPWM.on()
    M2_LPWM.off()


def motor2_reverse():
    M2_RPWM.off()
    M2_LPWM.on()


def motor_stop():
    print("Motors: STOP")
    motor1_stop()
    motor2_stop()


# --------- STEPPER SETUP (gpiozero) ---------
STEPPER_STEP = OutputDevice(17, active_high=True, initial_value=False)
STEPPER_DIR = OutputDevice(27, active_high=True, initial_value=False)

STEPPER_STEP_DELAY = 0.0005
STEPPER_STEP_CHUNK = 20


def stepper_move(steps: int, direction: int):
    STEPPER_DIR.value = 1 if direction else 0
    for _ in range(abs(steps)):
        STEPPER_STEP.on()
        time.sleep(STEPPER_STEP_DELAY)
        STEPPER_STEP.off()
        time.sleep(STEPPER_STEP_DELAY)


# --------- PACKET PARSER (NEW: [START, payload, END]) ---------
def read_packets_stream(ser, debug_raw=False):
    """
    Reads a byte stream and yields payload_byte for frames:
      [0xAA, payload(1 byte), 0x55]

    If the serial link breaks, returns (caller should reconnect).
    """
    buf = bytearray()

    while True:
        try:
            chunk = ser.read(16)
        except (serial.SerialException, OSError) as e:
            print("[GLOVE] Serial exception while reading:", e)
            return

        if not chunk:
            yield None
            continue

        if debug_raw:
            print("RAW:", chunk.hex())

        buf.extend(chunk)

        while True:
            if len(buf) < 3:
                break

            try:
                start_idx = buf.index(START)
            except ValueError:
                buf.clear()
                break

            if start_idx + 3 > len(buf):
                if start_idx > 0:
                    del buf[:start_idx]
                break

            payload = buf[start_idx + 1]
            end = buf[start_idx + 2]

            if end != END:
                # drop this START and resync
                del buf[start_idx]
                continue

            del buf[:start_idx + 3]
            yield payload


# --------- DEMO MAPPING (easy to change later) ---------
def handle_payload(payload: int):
    """
    Decode payload byte from glove and drive motors/stepper/servo.
    This is intentionally a simple placeholder mapping you can change later.
    """
    flex = (payload >> 4) & 0x0F           # b7..b4
    roll_code = (payload >> 2) & 0x03      # b3..b2
    pitch_code = payload & 0x03            # b1..b0

    # Pretty prints
    payload_bits = "".join(str((payload >> i) & 1) for i in range(7, -1, -1))
    print(f"[GLOVE] payload=0x{payload:02X} bits={payload_bits}  flex=0b{flex:04b} roll={roll_code:02b} pitch={pitch_code:02b}")

    dist = measure_distance()
    if dist is not None:
        print(f"Distance: {dist} cm")
    else:
        print("Distance: None (no echo)")

    # --- Placeholder control scheme ---
    # FLEX bits:
    #   flex0 -> servo enable
    #   flex1 -> servo direction
    #   flex2 -> stepper forward trigger
    #   flex3 -> stepper reverse trigger
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # Servo
    if f0 == 0:
        servo_stop()
    else:
        servo_spin(f1)

# Stepper
    if f2 == 1 and f3 == 0:
        print(f"Stepper: FORWARD {STEPPER_STEP_CHUNK} steps")
        stepper_move(STEPPER_STEP_CHUNK, direction=1)
    elif f3 == 1 and f2 == 0:
        print(f"Stepper: REVERSE {STEPPER_STEP_CHUNK} steps")
        stepper_move(STEPPER_STEP_CHUNK, direction=0)
    else:
        print("Stepper: no movement")

    # DC motors from IMU (roll/pitch codes)
    # roll_code: 01 = roll right, 10 = roll left, else neutral
    # pitch_code: 01 = tilt forward, 10 = tilt backward, else neutral
    #
    # Simple mapping:
    #   - forward/backward uses pitch
    #   - turning uses roll (differential)
    #
    # You can change this however you want.
    def blocked():
        # keep your old "disable distance check" behavior by default
        return False
        # if dist is not None and dist < THRESHOLD_CM:
        #     print(f"Obstacle {dist} cm < {THRESHOLD_CM} cm: blocking DC motion.")
        #     return True
        # return False

    if blocked():
        motor1_stop()
        motor2_stop()
        return

    if pitch_code == 0b01:  # forward
        if roll_code == 0b01:          # forward + turn right
            print("DC: forward + right")
            motor1_forward()
            motor2_stop()
        elif roll_code == 0b10:        # forward + turn left
            print("DC: forward + left")
            motor1_stop()
            motor2_forward()
        else:                          # straight forward
            print("DC: forward")
            motor1_forward()
            motor2_forward()

    elif pitch_code == 0b10:  # backward
        if roll_code == 0b01:
            print("DC: reverse + right")
            motor1_reverse()
            motor2_stop()
        elif roll_code == 0b10:
            print("DC: reverse + left")
            motor1_stop()
            motor2_reverse()
        else:
            print("DC: reverse")
            motor1_reverse()
            motor2_reverse()

    else:
        # neutral pitch => use roll to spin in place, otherwise stop
        if roll_code == 0b01:
            print("DC: spin right (in place)")
            motor1_forward()
            motor2_reverse()
        elif roll_code == 0b10:
            print("DC: spin left (in place)")
            motor1_reverse()
            motor2_forward()
        else:
            print("DC: stop")
            motor1_stop()
            motor2_stop()


def run_glove_loop():
    """
    Glove loop optimized for control (latest-state-wins):
    - read and parse as fast as possible
    - if multiple frames are waiting, DROP old ones and process only the newest
    - if link goes silent for a while, stop motors/servo for safety
    """
    print(f"[GLOVE] Using {RFCOMM_PORT} @ {GLOVE_BAUD} baud")
    print("[GLOVE] Listening to glove packets... (Ctrl+C to quit)")

    ser = None
    buf = bytearray()

    # Safety: if we haven't received a valid payload in this long -> stop outputs
    SILENCE_STOP_SEC = 0.7
    last_good_time = time.time()

    try:
        while True:
            if ser is None:
                ser = open_rfcomm_serial_forever(RFCOMM_PORT, GLOVE_BAUD)
                # Make reads more responsive (we can also do this inside open_rfcomm...)
                try:
                    ser.timeout = 0.05
                except Exception:
                    pass
                buf.clear()
                last_good_time = time.time()

            # Read *everything currently available* (fast drain)
            try:
                n = ser.in_waiting
                chunk = ser.read(n if n > 0 else 1)  # read at least 1 byte (timeout handles no-data)
            except (serial.SerialException, OSError) as e:
                print("[GLOVE] Serial exception while reading:", e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.2)
                continue

            if chunk:
                buf.extend(chunk)

            newest_payload = None

# Parse all complete frames currently in buf.
            # Frame: [START, payload, END]
            while True:
                if len(buf) < 3:
                    break

                try:
                    start_idx = buf.index(START)
                except ValueError:
                    buf.clear()
                    break

                # Not enough bytes for full frame yet
                if start_idx + 3 > len(buf):
                    # Keep from START onwards
                    if start_idx > 0:
                        del buf[:start_idx]
                    break

                payload = buf[start_idx + 1]
                end = buf[start_idx + 2]

                if end != END:
                    # bad alignment: drop this START and resync
                    del buf[start_idx]
                    continue

                # Valid frame -> consume it
                del buf[:start_idx + 3]
                newest_payload = payload  # keep overwriting; last one wins

            if newest_payload is not None:
                last_good_time = time.time()
                handle_payload(newest_payload)
            else:
                # No complete frame this iteration -> safety stop if silent too long
                if (time.time() - last_good_time) > SILENCE_STOP_SEC:
                    motor_stop()
                    servo_stop()
                    # Don't spam STOP prints; update last_good_time to rate-limit stops
                    last_good_time = time.time()

    except KeyboardInterrupt:
        print("\n[GLOVE] KeyboardInterrupt, exiting...")

    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass



def keyboard_control_loop():
    """
    Keyboard mode now supports either:
      1) payload byte in hex:   e.g.  A3
      2) payload bits (8):      e.g. 10100011
      3) old-style convenience: d0d1 s0s1 [ed]   (kept, but mapped into a fake payload)

    Recommended: just type 8 bits or hex for payload.
    """
    print("Keyboard mode enabled.")
    print("Enter one of:")
    print("  - payload HEX byte:   e.g.  A3")
    print("  - payload 8 bits:     e.g.  10100011")
    print("  - legacy: d0d1 s0s1 [ed] (will be converted into a payload)")
    print("Type 'q' to quit.\n")

    while True:
        line = input("Command: ").strip()
        if not line:
            continue

        if line.lower() in ("q", "quit", "exit"):
            print("Exiting keyboard mode.")
            break

        parts = line.split()

        # Case 1: single token HEX or 8-bit string
        if len(parts) == 1:
            token = parts[0].strip()

            # 8 bits
            if len(token) == 8 and all(c in "01" for c in token):
                payload = int(token, 2)
                handle_payload(payload)
                continue

            # hex byte (allow 0x prefix)
            try:
                payload = int(token, 16)
                if 0 <= payload <= 0xFF:
                    handle_payload(payload)
                    continue
            except ValueError:
                pass

            print("Invalid input. Use HEX byte (e.g. A3) or 8 bits (e.g. 10100011).")
            continue

        # Case 2: legacy format (kept from your old version)
        if len(parts) not in (2, 3):
            print("Invalid format.")
            continue

        dc_bits_str, step_bits_str = parts[0], parts[1]
        servo_bits_str = parts[2] if len(parts) == 3 else "00"

        if (len(dc_bits_str) != 2 or len(step_bits_str) != 2 or len(servo_bits_str) != 2 or
                any(c not in "01" for c in (dc_bits_str + step_bits_str + servo_bits_str))):
            print("Invalid legacy bits. Use only 0/1, with lengths: 2 2 [2].")
            continue

        # Convert legacy into a "fake" payload:
        # flex bits = [servo_enable, servo_dir, step_forward, step_reverse]
        se = int(servo_bits_str[0])
        sd = int(servo_bits_str[1])
        s0 = int(step_bits_str[0])
        s1 = int(step_bits_str[1])
        # step mapping from 2-bit: 01 forward, 10 reverse
        step_fwd = 1 if (s0, s1) == (0, 1) else 0
        step_rev = 1 if (s0, s1) == (1, 0) else 0

        flex = (se << 0) | (sd << 1) | (step_fwd << 2) | (step_rev << 3)

        # map DC 2-bit into pitch/roll codes in a simple way
        d0 = int(dc_bits_str[0])
        d1 = int(dc_bits_str[1])
        # 00 stop, 11 forward, 10 spin left, 01 spin right
        if (d0, d1) == (0, 0):
            roll_code, pitch_code = 0b00, 0b00
        elif (d0, d1) == (1, 1):
            roll_code, pitch_code = 0b00, 0b01
        elif (d0, d1) == (1, 0):
            roll_code, pitch_code = 0b10, 0b00
        else:  # (0,1)
            roll_code, pitch_code = 0b01, 0b00

        payload = ((flex & 0x0F) << 4) | ((roll_code & 0x03) << 2) | (pitch_code & 0x03)
        handle_payload(payload)


def main():
    print(f"INPUT MODE: {MODE}")
    servo_init()

    try:
        if MODE.upper() == "GLOVE":
            run_glove_loop()
        elif MODE.upper() == "KEYBOARD":
            keyboard_control_loop()
        else:
            print(f"Unknown MODE='{MODE}'. Use 'GLOVE' or 'KEYBOARD'.")
            return

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, exiting...")

    finally:
        print("Stopping motors and cleaning up...")
        motor_stop()
        servo_cleanup()
        GPIO.cleanup()


if name == "main":
    main()