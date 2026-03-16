#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Glove Controller (UDP) & Autonomy Mode
Handles UDP Glove commands, Ultrasonic Collision Avoidance, and YOLO Vision.
"""

import os
# Setting environment variables must happen before pigpio is imported
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
import socket
import math
import pickle
from typing import Optional

import cv2
import numpy as np
import pigpio
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from ultralytics import YOLO

# ============================================================
# CONSTANTS & CONFIGURATION
# ============================================================

# --- System & Operation Mode ---
MODE = "GLOVE"
CONTROL_PERIOD_SEC = 0.01

# --- UDP Network Config ---
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55
UDP_TIMEOUT_SEC = 0.02
SILENCE_STOP_SEC = 0.7
PAYLOAD_BUFFER_SIZE = 1024

# --- Hardware Pins (BCM) ---
# LED Configuration (PNP BC327: HIGH = OFF, LOW = ON)
RED_LED_PIN = 26    # Red: Obstacle (Ultrasonic) + Failure indicator
GREEN_LED_PIN = 16  # Green: Object detection (Image processing) success

SERVO_PIN = 12

US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19

RIGHT_RPWM_PIN = 2
RIGHT_LPWM_PIN = 3
LEFT_RPWM_PIN = 20
LEFT_LPWM_PIN = 21

ARM_RPWM_PIN = 27
ARM_LPWM_PIN = 22

ENC_LEFT_A = 24
ENC_LEFT_B = 25

# --- Ultrasonic Settings ---
ULTRA_STOP_CM = 40.0
US_TIMEOUT_SEC = 0.03
ULTRA_MEASURE_PERIOD_SEC = 0.10
SPEED_OF_SOUND_DIVISOR = 17150.0  # Constant used to convert echo time to CM

# --- Robot Kinematics & Mechanics ---
WHEEL_CIRCUMFERENCE_CM = math.pi * 7.2
TRACK_WIDTH_CM = 39.0          # Distance between wheels for this specific configuration
TURN_ANGLE_OFFSET_RAD = math.radians(5) # Angle reduction offset for turns
DISTANCE_UNDERSHOOT_CM = 1.0   # Stop distance slightly before target to account for inertia
ARM_Z_OFFSET_CM = 22.0         # Physical Z-axis offset to target for the arm's reach

SPEED_DIFF_FACTOR = 1.04       # Multiplier for straight-line driving
ARC_SPEED_DIFF_FACTOR = 1.06   # Dedicated multiplier for arc turns
DEFAULT_TURN_SPEED = 0.2
DEFAULT_DRIVE_SPEED = 0.3
DEFAULT_REVERSE_SPEED = 0.3
AUTONOMY_TURN_SPEED = 0.4      # Specific turning speed used during visual autonomy

SERVO_MOVE_STEP = 0.39

# --- Vision & Camera Settings ---
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 15
DEFAULT_IMG_SIZE = (CAM_WIDTH, CAM_HEIGHT)

BOTTLE_CLASS_ID = 39
YOLO_CONF_THRESHOLD = 0.50
MATCH_Y_TOLERANCE = 10

AUTONOMY_SEARCH_TIMEOUT_SEC = 5.0 # Max time to loop detection attempts
LED_FAIL_TIMEOUT_SEC = 5.0        # How long to flash red LEDs upon failure

# ============================================================
# INITIALIZATION (GPIO & FACTORIES)
# ============================================================
factory = PiGPIOFactory()
pi_enc = pigpio.pi()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup Ultrasonic Pins
GPIO.setup(US1_TRIG, GPIO.OUT)
GPIO.setup(US1_ECHO, GPIO.IN)
GPIO.setup(US2_TRIG, GPIO.OUT)
GPIO.setup(US2_ECHO, GPIO.IN)

# Initialize LEDs (PNP: HIGH = OFF)
GPIO.setup(RED_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)

GPIO.output(US1_TRIG, GPIO.LOW)
GPIO.output(US2_TRIG, GPIO.LOW)

# ============================================================
# GLOBAL STATE
# ============================================================
# Glove history
_prev_f0 = _prev_f1 = _prev_f2 = _prev_f3 = 0
_arm_dir = 0
_last_pkt_t = 0.0
last_rx = 0.0

# Drive state history
_prev_drive_req = "STOP"
_prev_prev_drive_req = "STOP"

# Ultrasonic override logic
_ignore_ultra_active = False
_ignore_ultra_cmd = None

# Timer for red LED indicator in case of detection failure
_red_led_fail_until = 0.0

# Ultrasonic states
_last_ultra_t = 0.0
_last_distances = [None, None]

# Socket
sock = None

# ============================================================
# ULTRASONIC SENSORS
# ============================================================
def _measure_distance_cm(trig: int, echo: int) -> Optional[float]:
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    t0 = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - t0 > US_TIMEOUT_SEC: 
            return None
            
    ps = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - ps > US_TIMEOUT_SEC: 
            return None
            
    pe = time.time()
    return (pe - ps) * SPEED_OF_SOUND_DIVISOR

def ultrasonic_tick() -> None:
    global _last_ultra_t, _last_distances
    now = time.time()
    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        _last_distances = [
            _measure_distance_cm(US1_TRIG, US1_ECHO),
            _measure_distance_cm(US2_TRIG, US2_ECHO),
        ]

def obstacle_too_close() -> bool:
    for d in _last_distances:
        if d is not None and d < ULTRA_STOP_CM:
            return True
    return False

# ============================================================
# SERVO & MOTORS
# ============================================================
servo = Servo(
    SERVO_PIN,
    initial_value=None,
    min_pulse_width=0.5 / 1000,
    max_pulse_width=2.5 / 1000,
    pin_factory=factory
)

_current_servo_pos = 0.0
_servo_is_open = True

def servo_move_step(direction: int) -> None:
    global _current_servo_pos, _servo_is_open
    want_close = (direction == 1)
    
    if want_close and not _servo_is_open: 
        return
    if not want_close and _servo_is_open: 
        return

    if want_close:
        new_val = _current_servo_pos + SERVO_MOVE_STEP
    else:
        new_val = _current_servo_pos - SERVO_MOVE_STEP

    new_val = max(-1.0, min(1.0, new_val))
    _current_servo_pos = new_val
    servo.value = _current_servo_pos
    _servo_is_open = not want_close

RIGHT_RPWM = PWMOutputDevice(RIGHT_RPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(RIGHT_LPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(LEFT_RPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(LEFT_LPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)

ARM_RPWM = PWMOutputDevice(ARM_RPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)
ARM_LPWM = PWMOutputDevice(ARM_LPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)

def stop_drive() -> None:
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0

def drive_forward(speed: float = DEFAULT_DRIVE_SPEED) -> None:
    RIGHT_RPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0

def drive_reverse(speed: float = DEFAULT_REVERSE_SPEED) -> None:
    RIGHT_LPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0

def turn_right(speed: float = DEFAULT_TURN_SPEED) -> None:
    RIGHT_LPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0
    
def turn_left(speed: float = DEFAULT_TURN_SPEED) -> None:
    RIGHT_RPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0

def run_arm(forward: bool) -> None:
    if forward:
        ARM_LPWM.value, ARM_RPWM.value = 1, 0
    else:
        ARM_LPWM.value, ARM_RPWM.value = 0, 1

def stop_arm() -> None:
    ARM_RPWM.value, ARM_LPWM.value = 0, 0

# ============================================================
# ENCODER
# ============================================================
class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1, 
              0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi: pigpio.pi, gpio_a: int, gpio_b: int):
        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.ppr_output = 384.5 * (100 / 106)
        self.counts_per_rev_output = self.ppr_output
        self.counts = 0
        self._last_state = (pi.read(gpio_a) << 1) | pi.read(gpio_b)
        
        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_mode(gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)
        
        self._cba = pi.callback(gpio_a, pigpio.EITHER_EDGE, self._cb)
        self._cbb = pi.callback(gpio_b, pigpio.EITHER_EDGE, self._cb)
        self._t_last = time.time()
        self._c_last = 0
        self._output_rpm = 0.0

    def _cb(self, gpio, level, tick) -> None:
        new_state = (self.pi.read(self.gpio_a) << 1) | self.pi.read(self.gpio_b)
        self.counts += self._TRANS.get((self._last_state << 2) | new_state, 0)
        self._last_state = new_state

    def update(self, window_s: float = 0.2) -> None:
        dt = time.time() - self._t_last
        if dt < window_s: 
            return
        self._output_rpm = 60.0 * ((self.counts - self._c_last) / self.counts_per_rev_output) / dt
        self._t_last, self._c_last = time.time(), self.counts

    def output_revolutions(self) -> float: 
        return self.counts / self.counts_per_rev_output

    def zero(self) -> None: 
        self.counts = self._c_last = 0

# NOTE: This script utilizes only the left encoder
enc = YellowJacketEncoder(pi_enc, ENC_LEFT_A, ENC_LEFT_B)

# ============================================================
# VISION
# ============================================================
_vision_ready = False
_model = None
_cap_l = None
_cap_r = None
_maps_l = None
_maps_r = None
_Q = None

def init_vision() -> None:
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    if _vision_ready: 
        return
        
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')
    _cap_l = cv2.VideoCapture(LEFT_CAM_PATH, cv2.CAP_V4L2)
    _cap_r = cv2.VideoCapture(RIGHT_CAM_PATH, cv2.CAP_V4L2)
    
    with open(CALIBRATION_FILE_PATH, 'rb') as f: 
        data = pickle.load(f)
        
    _Q = data['Q']
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        data['cameraMatrix1'], data['distCoeffs1'], 
        data['cameraMatrix2'], data['distCoeffs2'], 
        DEFAULT_IMG_SIZE, data['R'], data['T'], alpha=-1
    )
    
    _maps_l = cv2.initUndistortRectifyMap(data['cameraMatrix1'], data['distCoeffs1'], R1, P1, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    _maps_r = cv2.initUndistortRectifyMap(data['cameraMatrix2'], data['distCoeffs2'], R2, P2, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    _vision_ready = True

def detect_bottle_once() -> dict:
    global _model, _cap_l, _cap_r, _maps_l, _maps_r
    
    for _ in range(5): 
        _cap_l.grab()
        _cap_r.grab()
        
    _, fl = _cap_l.retrieve()
    _, fr = _cap_r.retrieve()
    f_rect = cv2.remap(fl, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    res = _model.predict(f_rect, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)
    
    X = Z = None
    if res and len(res[0].boxes) > 0:
        # Placeholder for real depth values as originally written
        X, Y, Z = 0, 0, 100  
        
    return {"found": X is not None, "X": X, "Z": Z}

# ============================================================
# MOTION & AUTONOMY
# ============================================================
def drive_distance(d_cm: float, forward: bool) -> None:
    enc.zero()
    enc.update()
    
    if forward:
        drive_forward()
    else:
        drive_reverse()
        
    while abs(enc.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM < (d_cm - DISTANCE_UNDERSHOOT_CM):
        time.sleep(0.01)
        enc.update()
        
    stop_drive()

def turn_angle(theta_rad: float, left_turn: bool) -> None:
    enc.zero()
    enc.update()
    
    segment_length = (TRACK_WIDTH_CM / 2.0) * abs(theta_rad - TURN_ANGLE_OFFSET_RAD)
    
    if left_turn:
        turn_left(AUTONOMY_TURN_SPEED)
    else:
        turn_right(AUTONOMY_TURN_SPEED)
        
    while abs(enc.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM < segment_length:
        time.sleep(0.01)
        enc.update()
        
    stop_drive()

def bring_bottle_xz() -> None:
    global _red_led_fail_until
    print("[AUTO] Starting detection loop...")

    # Ensure gripper is open before moving
    servo_move_step(0)
    time.sleep(1)

    start_time = time.time()
    found_bottle = {"found": False}

    # Detection attempt loop to minimize false negatives
    while (time.time() - start_time) < AUTONOMY_SEARCH_TIMEOUT_SEC:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            break
        time.sleep(0.1)

    if found_bottle["found"]:
        # Success: Turn on green LEDs
        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        time.sleep(1)

        # Turn off green LEDs just before driving
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)

        # Movement and grab sequence
        alpha = math.atan2(found_bottle['X'], ARM_Z_OFFSET_CM + found_bottle['Z'])
        turn_angle(alpha, alpha < 0)
        drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), True)

        time.sleep(1)
        servo_move_step(1)  # Close gripper
        time.sleep(1)

        # Raise arm 30 degrees (170 steps placeholder)
        # _stepper.step_chunk(170, direction=1, delay_sec=STEPPER_STEP_DELAY_SEC)
        time.sleep(0.5)

        drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), False)

        # Lower arm back
        # _stepper.step_chunk(170, direction=-1, delay_sec=STEPPER_STEP_DELAY_SEC)
        time.sleep(0.5)

        servo_move_step(0)  # Release
        time.sleep(1)
    else:
        # Failure: Set timer for red LEDs
        print("[AUTO] Timeout: Bottle not found.")
        _red_led_fail_until = time.time() + LED_FAIL_TIMEOUT_SEC

# ============================================================
# MAIN GLOVE LOOP
# ============================================================
def handle_payload(payload: int) -> None:
    global _prev_f0, _prev_f1, _prev_f2, _prev_f3, _arm_dir, _last_pkt_t
    global _prev_drive_req, _prev_prev_drive_req, _ignore_ultra_active, _ignore_ultra_cmd

    _last_pkt_t = time.time()

    # Extract finger bits via bitwise shift
    flex = (payload >> 4) & 0x0F
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # ---- Autonomy Trigger ----
    if (f0 == 1 and f1 == 1 and f2 == 1 and f3 == 1) and not (_prev_f0 == 1 and _prev_f1 == 1 and _prev_f2 == 1 and _prev_f3 == 1):
        bring_bottle_xz()

    # ---- Servo Edge Detection ----
    if f3 == 1 and _prev_f3 == 0:
        servo_move_step(1)  # Close
    if f2 == 1 and _prev_f2 == 0:
        servo_move_step(0)  # Open

    # ---- Stepper Held Command ----
    if f0 == 1 and f1 == 0:
        _arm_dir = -1
    elif f1 == 1 and f0 == 0:
        _arm_dir = +1
    else:
        _arm_dir = 0

    # ---- Update Finger History for Edge Detection ----
    _prev_f0, _prev_f1, _prev_f2, _prev_f3 = f0, f1, f2, f3

    # ---- Decode Drive Request ----
    pitch_code = payload & 0x03
    roll_code = (payload >> 2) & 0x03

    if pitch_code == 0b01:
        drive_req = "REV"
    elif pitch_code == 0b10:
        drive_req = "FWD"
    elif roll_code == 0b01:
        drive_req = "LEFT"
    elif roll_code == 0b10:
        drive_req = "RIGHT"
    else:
        drive_req = "STOP"

    print(f"Received drive request: {drive_req} (payload: {payload:08b})")

    too_close = obstacle_too_close()

    # ------------------------------------------------------------
    # Ignore-Ultrasonic Logic (Double-Tap bypass):
    # Arm when we see: X, STOP, X   (X in {FWD, LEFT, RIGHT})
    # Keep active only while holding the same X.
    # Disable when STOP or REV or direction change.
    # ------------------------------------------------------------
    allowed_dir = {"FWD", "LEFT", "RIGHT"}

    # Disable ignore mode if STOP/REV or changed direction
    if _ignore_ultra_active:
        if drive_req in ("STOP", "REV") or drive_req != _ignore_ultra_cmd:
            _ignore_ultra_active = False
            _ignore_ultra_cmd = None

    # Arm ignore mode on pattern X, STOP, X
    if (not _ignore_ultra_active) and (drive_req in allowed_dir):
        if _prev_drive_req == "STOP" and _prev_prev_drive_req == drive_req:
            _ignore_ultra_active = True
            _ignore_ultra_cmd = drive_req
            print(f"[MODE] Ignoring ultrasonics while holding {drive_req}")

    ignore_ultra_now = (_ignore_ultra_active and drive_req == _ignore_ultra_cmd)

    # ---- Apply Drive (Reverse always allowed) ----
    if drive_req == "REV":
        drive_reverse()

    elif drive_req == "FWD":
        if (not too_close) or ignore_ultra_now:
            drive_forward()
        else:
            stop_drive()

    elif drive_req == "LEFT":
        if (not too_close) or ignore_ultra_now:
            turn_left()
        else:
            stop_drive()

    elif drive_req == "RIGHT":
        if (not too_close) or ignore_ultra_now:
            turn_right()
        else:
            stop_drive()

    else:
        stop_drive()

    # ---- Update Drive History (Requested Commands) ----
    _prev_prev_drive_req = _prev_drive_req
    _prev_drive_req = drive_req

def run_glove_loop() -> None:
    global _arm_dir, _last_pkt_t, sock, last_rx, _red_led_fail_until
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(UDP_TIMEOUT_SEC)
    
    last_rx = _last_pkt_t = time.time()
    print(f"debug: Listening for UDP packets on {UDP_LISTEN_IP}:{UDP_PORT}...")
    
    while True:
        ultrasonic_tick()

        now = time.time()
        too_close = obstacle_too_close()
        
        # Red LED ON if obstacle detected OR failure timer is active
        fail_active = (now < _red_led_fail_until)

        if too_close or fail_active:
            GPIO.output(RED_LED_PIN, GPIO.LOW)   # ON (PNP)
        else:
            GPIO.output(RED_LED_PIN, GPIO.HIGH)  # OFF

        if _arm_dir != 0: 
            run_arm(_arm_dir == 1)
        else: 
            stop_arm()

        try:
            data, _ = sock.recvfrom(PAYLOAD_BUFFER_SIZE)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC: 
                stop_drive()

        time.sleep(CONTROL_PERIOD_SEC)

if __name__ == "__main__":
    try:
        init_vision()
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_drive()
        GPIO.output(RED_LED_PIN, GPIO.HIGH)
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")