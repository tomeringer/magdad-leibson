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
DEBUG_MODE = True
CONTROL_PERIOD_SEC = 0.01
BOTTLE_SEARCH_TIMEOUT_SEC = 10.0  # Maximum time to look for the bottle before aborting

# --- UDP Network Config ---
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55
UDP_TIMEOUT_SEC = 0.02
SILENCE_STOP_SEC = 0.7
PAYLOAD_BUFFER_SIZE = 1024

# --- Hardware Pins (BCM) ---

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
ENC_RIGHT_A = 9
ENC_RIGHT_B = 10

STEPPER_PINS = [23, 22, 27, 17]

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
ARC_MAX_OUTER_SPEED = 0.3  # The constant speed for the outer wheel during arc turns

SERVO_MOVE_STEP = 0.39
STEPPER_STEP_DELAY_SEC = 0.0015

# --- Vision Math Offsets ---
# These compensate for physical camera placement relative to the robot's center
CAMERA_YAW_OFFSET_RAD = math.radians(3)  # 3-degree alignment offset
CAMERA_X_OFFSET_CM = 5.0                 # Physical X-axis offset of the camera
ARC_STOP_OFFSET_CM = 4.5                 # Distance to subtract from arc travel to prevent collisions
Z0_STOP_DISTANCE_CM = 21.0               # Ideal Z-distance to stop at before grabbing (z0)

# --- Vision & Camera Settings ---
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 5
DEFAULT_IMG_SIZE = (CAM_WIDTH, CAM_HEIGHT)

BOTTLE_CLASS_ID = 39
YOLO_CONF_THRESHOLD = 0.50
MATCH_Y_TOLERANCE = 10

AUTONOMY_SEARCH_TIMEOUT_SEC = 5.0 # Max time to loop detection attempts

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
# ENCODERS & STEPPER
# ============================================================
class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1, 
              0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi: pigpio.pi, gpio_a: int, gpio_b: int):
        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        # Base Pulses Per Revolution adjusted for mechanical gear ratio
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


enc_left = YellowJacketEncoder(pi_enc, ENC_LEFT_A, ENC_LEFT_B)
enc_right = YellowJacketEncoder(pi_enc, ENC_RIGHT_A, ENC_RIGHT_B)

class StepperMotor:
    def __init__(self, pins: list):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [[1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]]
        for p in self.pins: 
            self.pi.set_mode(p, pigpio.OUTPUT)

    def step_chunk(self, steps: int, direction: int = 1, delay_sec: float = STEPPER_STEP_DELAY_SEC) -> None:
        direction = 1 if direction >= 0 else -1
        for _ in range(int(steps)):
            for step in range(4):
                idx = step if direction == 1 else 3 - step
                for i, p in enumerate(self.pins): 
                    self.pi.write(p, self.seq[idx][i])
                time.sleep(delay_sec)
        self.deenergize()

    def deenergize(self) -> None:
        for p in self.pins: 
            self.pi.write(p, 0)

    def close(self) -> None:
        self.deenergize()
        self.pi.stop()

_stepper = StepperMotor(STEPPER_PINS)


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

def open_cam(path: str, name: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"[CAM] {name}: opening {path} isOpened={cap.isOpened()}", flush=True)
    if not cap.isOpened():
        return cap

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(10):
        cap.read()

    print(f"[CAM] {name}: negotiated "
          f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.1f}fps", flush=True)
    return cap

def load_and_prepare_calibration(file_path: str):
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)

        K1, D1 = data['cameraMatrix1'], data['distCoeffs1']
        K2, D2 = data['cameraMatrix2'], data['distCoeffs2']
        R, T, Q = data['R'], data['T'], data['Q']
    except Exception as e:
        raise RuntimeError(f"Calibration load failed: {e}")

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=DEFAULT_IMG_SIZE, R=R, T=T,
        alpha=-1
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    return (map1_l, map2_l), (map1_r, map2_r), Q

def calculate_3d_coords(disparity: float, x: int, y: int, Q_matrix: np.ndarray) -> tuple:
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    hp = np.dot(Q_matrix, point)
    w = hp[3]
    if w == 0:
        return (0, 0, 0)
    return (hp[0] / w, hp[1] / w, hp[2] / w)

def hard_reset_usb_all() -> None:
    """Forces the entire USB hub to reset (software equivalent of replugging)."""
    print("[SYSTEM] Starting mandatory USB reset for stability...", flush=True)
    try:
        os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null")
        time.sleep(2)
        os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null")
        print("[SYSTEM] USB Hub reset successful. Waiting for device enumeration...", flush=True)
        time.sleep(4) 
    except Exception as e:
        print(f"[ERROR] Could not perform USB reset: {e}")

def hard_reset_usb() -> None:
    """Targeted reset for specific camera USB ports."""
    camera_ports = ['1-1.3', '1-1.4']
    
    print(f"[SYSTEM] Resetting specific camera ports: {camera_ports}...", flush=True)
    
    for port in camera_ports:
        try:
            os.system(f"echo '{port}' | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null")
            print(f"  -> Port {port} unbound.")
        except Exception: 
            pass

    time.sleep(2) 

    for port in camera_ports:
        try:
            os.system(f"echo '{port}' | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null")
            print(f"  -> Port {port} bound.")
        except Exception: 
            pass
        
    print("[SYSTEM] Specific reset complete. Waiting for recovery...", flush=True)
    time.sleep(3)

def init_vision() -> None:
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    if _vision_ready:
        return

    hard_reset_usb_all()

    print("[VISION] Initializing YOLO + stereo...", flush=True)
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')

    _cap_l = open_cam(LEFT_CAM_PATH, "LEFT")
    _cap_r = open_cam(RIGHT_CAM_PATH, "RIGHT")
    if not _cap_l.isOpened() or not _cap_r.isOpened():
        raise RuntimeError("One or both cameras failed to open.")

    _maps_l, _maps_r, _Q = load_and_prepare_calibration(CALIBRATION_FILE_PATH)
    _vision_ready = True
    print("[VISION] Ready.", flush=True)

def read_latest_stereo(cap_l: cv2.VideoCapture, cap_r: cv2.VideoCapture, flush_n: int = 10) -> tuple:
    for _ in range(flush_n):
        cap_l.grab()
        cap_r.grab()
    ret_l, frame_l = cap_l.retrieve()
    ret_r, frame_r = cap_r.retrieve()
    return ret_l, frame_l, ret_r, frame_r

def detect_bottle_once() -> dict:
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    t0 = time.perf_counter()
    ret_l, frame_l_orig, ret_r, frame_r_orig = read_latest_stereo(_cap_l, _cap_r, flush_n=5)

    if not ret_l or not ret_r:
        return {"found": False, "err": "Failed to read frames", "t_proc": time.perf_counter() - t0}

    frame_l = cv2.remap(frame_l_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    frame_r = cv2.remap(frame_r_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

    results_l = _model.predict(frame_l, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)
    results_r = _model.predict(frame_r, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)

    X = Y = Z = cx = cy = disparity = conf = None

    if results_l and len(results_l[0].boxes) > 0:
        box_l = results_l[0].boxes[0]
        cx = int(box_l.xywh[0][0].item())
        cy = int(box_l.xywh[0][1].item())
        conf = float(box_l.conf[0].item()) if hasattr(box_l, "conf") else None

        closest_box_r = None
        if results_r and len(results_r[0].boxes) > 0:
            for box_r in results_r[0].boxes:
                x_r = int(box_r.xywh[0][0].item())
                y_r = int(box_r.xywh[0][1].item())
                if abs(cy - y_r) < MATCH_Y_TOLERANCE:
                    closest_box_r = box_r
                    break

        if closest_box_r is not None:
            x_r = int(closest_box_r.xywh[0][0].item())
            disparity = abs(cx - x_r)

            if disparity > 0:
                X, Y, Z_raw = calculate_3d_coords(disparity, cx, cy, _Q)
                
                # Trigonometric adjustments based on physical camera placement
                alpha = math.atan2(X, Z_raw)
                r = math.hypot(X, Z_raw)
                alpha = alpha + CAMERA_YAW_OFFSET_RAD
                X = math.sin(alpha) * r - CAMERA_X_OFFSET_CM 
                Z = math.cos(alpha) * r

    return {
        "found": X is not None,
        "X": X, "Y": Y, "Z": Z,
        "cx": cx, "cy": cy,
        "disparity": disparity,
        "conf": conf,
        "t_proc": time.perf_counter() - t0
    }

def shutdown_vision() -> None:
    global _cap_l, _cap_r
    if _cap_l is not None: _cap_l.release()
    if _cap_r is not None: _cap_r.release()

# ============================================================
# MOTION & AUTONOMY
# ============================================================

def get_average_distance() -> float:
    dist_left = abs(enc_left.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM
    dist_right = abs(enc_right.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM
    return (dist_left + dist_right) / 2.0

def drive_distance(d_cm: float, forward: bool) -> None:
    enc_left.zero()
    enc_right.zero()
    enc_left.update()
    enc_right.update()
    
    # Targeting 1cm less than requested to account for momentum/inertia
    target_dist = d_cm - 1 
    
    if forward:
        drive_forward(DEFAULT_DRIVE_SPEED)
    else:
        drive_reverse(DEFAULT_REVERSE_SPEED)
        
    current_dist = 0.0
    while current_dist < target_dist:
        enc_left.update()
        enc_right.update()
        current_dist = get_average_distance()
        
        if DEBUG_MODE:
            print(f"Distance traveled: R={abs(enc_right.output_revolutions()):.3f}, "
                  f"L={abs(enc_left.output_revolutions()):.3f}")
        time.sleep(0.01)
        
    stop_drive()

def turn_angle(theta_rad: float, left_turn: bool) -> None:
    enc_left.zero()
    enc_right.zero()
    enc_left.update()
    enc_right.update()
    
    radius = TRACK_WIDTH_CM / 2.0
    segment_length = radius * abs(theta_rad)
    
    if left_turn:
        turn_left(DEFAULT_TURN_SPEED)
    else:
        turn_right(DEFAULT_TURN_SPEED)
        
    current_dist = 0.0
    while current_dist < segment_length:
        enc_left.update()
        enc_right.update()
        current_dist = get_average_distance()
        
        if DEBUG_MODE:
            print(f"Distance traveled: {current_dist:.2f}")
        time.sleep(0.01)
        
    stop_drive()

def drive_arc(target_x: float, target_z: float) -> None:
    # 1. Calculate the radius of the arc
    R = (target_x ** 2 + target_z ** 2) / (2 * abs(target_x))

    # 2. Define wheel radii
    r_inner = R - (TRACK_WIDTH_CM / 2)
    r_outer = R + (TRACK_WIDTH_CM / 2)

    # 3. Calculate Speed Ratio
    speed_inner = ARC_MAX_OUTER_SPEED * (r_inner / r_outer)

    # 4. Determine kinematics based on direction
    if target_x > 0:  # Turning Right
        v_left = ARC_MAX_OUTER_SPEED
        v_right = speed_inner
    else:  # Turning Left
        v_left = speed_inner
        v_right = ARC_MAX_OUTER_SPEED

    # 5. Calculate target distance
    angle = math.atan2(target_z, R - abs(target_x))
    total_arc_length = R * angle - ARC_STOP_OFFSET_CM

    RIGHT_RPWM.value, LEFT_RPWM.value = v_right * ARC_SPEED_DIFF_FACTOR, v_left
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0

    # 6. Monitor distance
    enc_left.zero()
    enc_right.zero()
    current_dist = 0.0
    
    while current_dist < total_arc_length:
        enc_left.update()
        enc_right.update()
        current_dist = get_average_distance()
        time.sleep(0.01)

    stop_drive()

def bring_bottle_xz() -> None:
    time.sleep(2)
    
    z0 = Z0_STOP_DISTANCE_CM 
    t0 = time.perf_counter()
    found_bottle = {"found": False}

    while True:
        found_bottle = detect_bottle_once()
        now = time.perf_counter()
        
        if now - t0 > BOTTLE_SEARCH_TIMEOUT_SEC:
            print("Timeout: No bottle found.")
            return
            
        if found_bottle["found"]:
            print(f"Targeting bottle at X={found_bottle['X']:.2f}, Z={found_bottle['Z']:.2f}")
            break
        else:
            print("No bottle detected.")
            time.sleep(0.1)

    drive_arc(found_bottle['X'], found_bottle['Z'])
    print("Arrived at bottle location via arc.")
    time.sleep(1)
    servo_move_step(0)

def track_bottle_continuous() -> None:
    print("\n[VISION] Starting continuous tracking. Press Ctrl+C to return to menu.")
    try:
        while True:
            bottle = detect_bottle_once()
            if bottle["found"]:
                print(f"Bottle Location -> X: {bottle['X']:.2f}, Y: {bottle['Y']:.2f}, Z: {bottle['Z']:.2f}")
            else:
                print("Bottle not found...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[VISION] Tracking stopped. Returning to menu.")

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
    global _arm_dir, _last_pkt_t, sock, last_rx
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(UDP_TIMEOUT_SEC)
    
    last_rx = _last_pkt_t = time.time()
    print(f"debug: Listening for UDP packets on {UDP_LISTEN_IP}:{UDP_PORT}...")
    
    while True:
        ultrasonic_tick()

        now = time.time()
        too_close = obstacle_too_close()

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
        shutdown_vision()
        _stepper.close()
        GPIO.cleanup()
        print("\n[OFF] System Stopped.")