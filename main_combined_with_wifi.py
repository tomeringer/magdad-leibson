#!/usr/bin/env python3
import os, time, math, sys, tty, termios, pigpio
import serial
import socket
import select
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

import chassis
import gripper
import arm
import vision
import piano_player

os.environ['PIGPIO_ADDR'] = 'localhost'

# ========================================================
# CONSTANTS & CONFIGURATION
# ========================================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

UDP_IP = "0.0.0.0"
UDP_PORT = 4210

ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06

# --- LED GPIO PINS ---
LED_R_PIN = 18
LED_G_PIN = 23
LED_B_PIN = 8

factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

_arm_dir = 0
_prev_f = [0, 0, 0, 0, 0]
_binary_f = [0, 0, 0, 0, 0]  # Tracks the latched binary states of the fingers
_drive_hist = ["STOP", "STOP"]
_ignore_ultra = [False, None]
last_rx = 0.0


# ========================================================
# LED STATUS SYSTEM
# ========================================================
def init_leds():
    """Initializes the LED PWM using the existing pigpio instance."""
    try:
        pi_enc.set_PWM_range(LED_R_PIN, 255)
        pi_enc.set_PWM_range(LED_G_PIN, 255)
        pi_enc.set_PWM_range(LED_B_PIN, 255)
        set_led_color(0, 0, 0)  # Ensure they start OFF
    except Exception as e:
        print(f"[LED ERROR] Failed to initialize LEDs: {e}")


def set_led_color(r, g, b):
    """Sets the RGB LED strip color (0-255 per channel)."""
    try:
        pi_enc.set_PWM_dutycycle(LED_R_PIN, r)
        pi_enc.set_PWM_dutycycle(LED_G_PIN, g)
        pi_enc.set_PWM_dutycycle(LED_B_PIN, b)
    except:
        pass


# ========================================================
# GRIPPER MODE
# ========================================================
def get_average_distance():
    dist_l = abs(chassis.enc_left.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    dist_r = abs(chassis.enc_right.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    return (dist_l + dist_r) / 2.0


def drive_arc(target_x, target_z):
    L = Z0_STOP_DISTANCE_CM
    X_b = target_x
    Z_b = target_z + L

    # Calculate center of rotation
    x_c = (X_b ** 2 + Z_b ** 2 - L ** 2) / (2 * X_b)
    R = abs(x_c)

    r_in = R - (chassis.TRACK_WIDTH_CM / 2)
    r_out = R + (chassis.TRACK_WIDTH_CM / 2)

    # Prevent negative radius if turn is tighter than wheelbase
    if r_in < 0: r_in = 0.1

    phi_start = math.atan2(L, -x_c)
    phi_end = math.atan2(Z_b, X_b - x_c)
    theta = abs(phi_end - phi_start)

    # 1. Stop when the OUTER wheel reaches its specific arc length
    target_dist_out = (r_out * theta) - ARC_STOP_OFFSET_CM

    # 2. The physical ratio the wheels MUST maintain
    ideal_ratio = r_in / r_out

    chassis.enc_left.zero()
    chassis.enc_right.zero()

    # The minimum PWM required to make your motors actually move.
    # Tune this if the inner wheel still stalls!
    MIN_PWM = 0.15

    v_out = ARC_MAX_OUTER_SPEED
    v_in_base = max(MIN_PWM, ARC_MAX_OUTER_SPEED * ideal_ratio)

    while True:
        dist_l = abs(chassis.enc_left.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
        dist_r = abs(chassis.enc_right.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM

        # Identify which wheel is out/in based on turn direction
        if target_x > 0:  # Turning right
            dist_out, dist_in = dist_l, dist_r
        else:  # Turning left
            dist_out, dist_in = dist_r, dist_l

        # Break strictly based on the outer wheel's progress
        if dist_out >= target_dist_out:
            break

        # --- DYNAMIC CORRECTION ---
        # Wait until we've moved at least 1cm to avoid dividing by near-zero
        if dist_out > 1.0:
            current_ratio = dist_in / dist_out
            error = ideal_ratio - current_ratio

            # P-Controller: Adjust inner speed.
            # If error is positive (inner wheel lagging), increase inner PWM.
            # '0.8' is the tuning gain. Increase it if corrections are too slow.
            v_in_adjusted = v_in_base + (error * 0.8)
        else:
            v_in_adjusted = v_in_base

        # Clamp the adjusted speed so we don't feed invalid PWM values
        v_in = max(MIN_PWM, min(ARC_MAX_OUTER_SPEED, v_in_adjusted))

        # Apply speeds
        if target_x > 0:  # Turn right
            chassis.LEFT_RPWM.value = v_out
            chassis.RIGHT_RPWM.value = v_in * ARC_SPEED_DIFF_FACTOR
        else:  # Turn left
            chassis.RIGHT_RPWM.value = v_out * ARC_SPEED_DIFF_FACTOR
            chassis.LEFT_RPWM.value = v_in

        chassis.LEFT_LPWM.value = chassis.RIGHT_LPWM.value = 0.0

        time.sleep(0.01)

    chassis.stop_drive()


def bring_bottle_xz():
    gripper.move_step(0)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 5.0:
        set_led_color(255, 128, 0)  # ORANGE: Searching for bottle

        res = vision.detect_bottle_once()
        if res["found"]:
            set_led_color(0, 255, 0)  # GREEN: Bottle found!
            drive_arc(res['X'], res['Z'])
            time.sleep(0.3)
            gripper.move_step(1)
            set_led_color(0, 0, 0)  # OFF: Finished task
            return
        time.sleep(0.1)

    set_led_color(0, 0, 0)  # OFF: Timeout, failed to find bottle


def handle_payload(merged_byte, flex_low, b3=0, b4=0):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _prev_f, _binary_f

    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03
    modeBit = (merged_byte >> 6) & 0x01

    # Extract raw 0-3 states
    f_2b = [(flex_low >> (i * 2)) & 0x03 for i in range(4)]
    f_2b.append((merged_byte >> 4) & 0x03)

    # FIXED: Software Hysteresis - Latch ON at 3, Latch OFF at 1 (or 0)
    for i in range(5):
        if _binary_f[i] == 0 and f_2b[i] == 3:
            _binary_f[i] = 1
        elif _binary_f[i] == 1 and f_2b[i] <= 1:
            _binary_f[i] = 0