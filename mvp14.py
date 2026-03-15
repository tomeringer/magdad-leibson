#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

os.environ['PIGPIO_ADDR'] = 'localhost'

import time
import socket
import pigpio
import RPi.GPIO as GPIO
from typing import Optional
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

import math
import pickle

import cv2
import numpy as np
from ultralytics import YOLO

# ============================================================
# CONFIGURATION
# ============================================================
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

# --- ×”×’×“×¨×•×ª LED (PNP BC327) ---
RED_LED_PIN = 26  # ××“×•×ž×™× - ×ž×›×©×•×œ (××•×œ×˜×¨×¡×•× ×™) + ×—×™×•×•×™ ×›×©×œ
GREEN_LED_PIN = 16  # ×™×¨×•×§×™× - ×–×™×”×•×™ ×—×¤×¥ (×¢×™×‘×•×“ ×ª×ž×•× ×”)
# ------------------------------

factory = PiGPIOFactory()

# ---- Stereo / YOLO config ----
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

W = 640
H = 480
CAM_FPS = 15
DEFAULT_IMG_SIZE = (W, H)

BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50
MATCH_Y_TOL = 10

# ---- Stepper config ----
STEPPER_CHUNK = 4
STEPPER_STEP_DELAY_SEC = 0.0015
STEPPER_TICK_SEC = 0.05
_last_stepper_cmd_t = 0.0
STEP_CMD_STALE_SEC = 0.12
_stepper_dir = 0
_last_pkt_t = 0.0
sock = None
last_rx = 0.0

# ============================================================
# GLOBAL STATE
# ============================================================
prev_f0 = prev_f1 = prev_f2 = prev_f3 = 0
_prev_drive_req = "STOP"
_prev_prev_drive_req = "STOP"
_ignore_ultra_active = False
_ignore_ultra_cmd = None

# ×˜×™×™×ž×¨ ×¢×‘×•×¨ ×—×™×•×•×™ ×”××“×•× ×‘×ž×§×¨×” ×©×œ ×›×©×œ ×‘×–×™×”×•×™
_red_led_fail_until = 0.0

# ============================================================
# ULTRASONIC SENSORS & GPIO SETUP
# ============================================================
US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(US1_TRIG, GPIO.OUT)
GPIO.setup(US1_ECHO, GPIO.IN)
GPIO.setup(US2_TRIG, GPIO.OUT)
GPIO.setup(US2_ECHO, GPIO.IN)

# ××ª×—×•×œ ×œ×“×™× (PNP: HIGH = OFF)
GPIO.setup(RED_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)

GPIO.output(US1_TRIG, GPIO.LOW)
GPIO.output(US2_TRIG, GPIO.LOW)

ULTRA_STOP_CM = 40.0
US_TIMEOUT_SEC = 0.03
ULTRA_MEASURE_PERIOD_SEC = 0.10

_last_ultra_t = 0.0
_last_distances = [None, None]


def _measure_distance_cm(trig, echo) -> Optional[float]:
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    t0 = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - t0 > US_TIMEOUT_SEC: return None
    ps = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - ps > US_TIMEOUT_SEC: return None
    pe = time.time()
    return (pe - ps) * 17150.0


def ultrasonic_tick():
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
SERVO_PIN = 12
servo = Servo(
    SERVO_PIN,
    initial_value=None,
    min_pulse_width=0.5 / 1000,
    max_pulse_width=2.5 / 1000,
    pin_factory=factory
)

MOVE_STEP = 0.39
current_pos = 0.0
servo_is_open = True


def servo_move_step(direction):
    global current_pos, servo_is_open
    want_close = (direction == 1)
    if want_close and (not servo_is_open): return
    if (not want_close) and servo_is_open: return

    if want_close:
        new_val = current_pos + MOVE_STEP
    else:
        new_val = current_pos - MOVE_STEP

    new_val = max(-1.0, min(1.0, new_val))
    current_pos = new_val
    servo.value = current_pos
    servo_is_open = (not want_close)


RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


def stop_drive():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


def drive_forward(speed: float = 0.5):
    RIGHT_RPWM.value, LEFT_RPWM.value = speed + 0.01, speed
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0


def drive_reverse():
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.51, 0.50
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0


def turn_right(speed: float = 0.5):
    RIGHT_LPWM.value, LEFT_RPWM.value = speed + 0.01, speed
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0


def turn_left(speed: float = 0.5):
    RIGHT_RPWM.value, LEFT_LPWM.value = speed + 0.01, speed
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0


# ============================================================
# ENCODER & STEPPER
# ============================================================
pi_enc = pigpio.pi()


class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1, 0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi, gpio_a, gpio_b):
        self.pi, self.gpio_a, self.gpio_b = pi, gpio_a, gpio_b
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
        self._t_last, self._c_last, self._output_rpm = time.time(), 0, 0.0

    def _cb(self, gpio, level, tick):
        new_state = (self.pi.read(self.gpio_a) << 1) | self.pi.read(self.gpio_b)
        self.counts += self._TRANS.get((self._last_state << 2) | new_state, 0)
        self._last_state = new_state

    def update(self, window_s=0.2):
        dt = time.time() - self._t_last
        if dt < window_s: return
        self._output_rpm = 60.0 * ((self.counts - self._c_last) / self.counts_per_rev_output) / dt
        self._t_last, self._c_last = time.time(), self.counts

    def output_revolutions(self): return self.counts / self.counts_per_rev_output

    def zero(self): self.counts = self._c_last = 0


enc = YellowJacketEncoder(pi_enc, 24, 25)


class StepperMotor:
    def __init__(self, pins):
        self.pins, self.pi = pins, pigpio.pi()
        self.seq = [
            [1, 0, 0, 0],  # A+ only
            [1, 0, 1, 0],  # A+ and B+
            [0, 0, 1, 0],  # B+ only
            [0, 1, 1, 0],  # A- and B+
            [0, 1, 0, 0],  # A- only
            [0, 1, 0, 1],  # A- and B-
            [0, 0, 0, 1],  # B- only
            [1, 0, 0, 1]  # A+ and B-
        ]
        for p in self.pins: self.pi.set_mode(p, pigpio.OUTPUT)

    def step_chunk(self, steps, direction=1, delay_sec=0.002):
        direction = 1 if direction >= 0 else -1
        for _ in range(int(steps)):
            for step in range(4):
                idx = step if direction == 1 else 3 - step
                for i, p in enumerate(self.pins): self.pi.write(p, self.seq[idx][i])
                time.sleep(delay_sec)
        self.deenergize()

    def deenergize(self):
        for p in self.pins: self.pi.write(p, 0)

    def close(self):
        self.deenergize()
        self.pi.stop()


_stepper = StepperMotor([23, 22, 27, 17])


def stepper_tick(direction: int):
    global _last_stepper_cmd_t
    if time.time() - _last_stepper_cmd_t >= STEPPER_TICK_SEC:
        _last_stepper_cmd_t = time.time()
        _stepper.step_chunk(STEPPER_CHUNK, direction=direction, delay_sec=STEPPER_STEP_DELAY_SEC)


# ============================================================
# VISION
# ============================================================
_vision_ready = False
_model = _cap_l = _cap_r = _maps_l = _maps_r = _Q = None


def open_cam(path: str, name: str):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"[CAM] {name}: opening {path} isOpened={cap.isOpened()}", flush=True)
    if not cap.isOpened():
        return cap

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    # Warm up
    for _ in range(10):
        cap.read()

    print(f"[CAM] {name}: negotiated "
          f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.1f}fps", flush=True)
    return cap


def load_and_prepare_calibration(file_path):
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)

        K1 = data['cameraMatrix1']
        D1 = data['distCoeffs1']
        K2 = data['cameraMatrix2']
        D2 = data['distCoeffs2']
        R = data['R']
        T = data['T']
        Q = data['Q']

    except Exception as e:
        raise RuntimeError(f"Calibration load failed: {e}")

    img_size = DEFAULT_IMG_SIZE
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=img_size, R=R, T=T,
        alpha=-1
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    return (map1_l, map2_l), (map1_r, map2_r), Q


def calculate_3d_coords(disparity, x, y, Q_matrix):
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    hp = np.dot(Q_matrix, point)
    w = hp[3]
    if w == 0:
        return (0, 0, 0)
    X = hp[0] / w - 5 # calibration to actual center between cams
    Y = hp[1] / w
    Z = hp[2] / w
    return (X, Y, Z)


def init_vision():
    """
    Call once at startup (or lazy-init on first 'c').
    """
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q

    if _vision_ready:
        return

    print("[VISION] Initializing YOLO + stereo...", flush=True)

    # Load YOLO (adjust to your actual model path)
    # _model = YOLO('yolov8n.pt')
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')

    _cap_l = open_cam(LEFT_CAM, "LEFT")
    _cap_r = open_cam(RIGHT_CAM, "RIGHT")
    if not _cap_l.isOpened() or not _cap_r.isOpened():
        raise RuntimeError("One or both cameras failed to open.")

    _maps_l, _maps_r, _Q = (*load_and_prepare_calibration(CALIBRATION_FILE_PATH),)
    # load_and_prepare_calibration returns (maps_l, maps_r, Q)
    # The line above packs it into _maps_l, _maps_r, _Q

    _vision_ready = True
    print("[VISION] Ready.", flush=True)


def read_latest_stereo(cap_l, cap_r, flush_n=10):
    """
    Flush old frames from both cameras and return the latest pair.
    """
    for _ in range(flush_n):
        cap_l.grab()
        cap_r.grab()

    ret_l, frame_l = cap_l.retrieve()
    ret_r, frame_r = cap_r.retrieve()

    return ret_l, frame_l, ret_r, frame_r


def detect_bottle_once():
    """
    Grabs one stereo pair and tries to compute bottle (X,Y,Z) and pixel center.
    Returns dict with keys: found, X,Y,Z,cx,cy,disparity,conf,t_proc
    """
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q

    t0 = time.perf_counter()

    ret_l, frame_l_orig, ret_r, frame_r_orig = read_latest_stereo(
        _cap_l, _cap_r, flush_n=5
    )

    if not ret_l or not ret_r:
        return {"found": False, "err": "Failed to read frames", "t_proc": time.perf_counter() - t0}

    # Rectify
    frame_l = cv2.remap(frame_l_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    frame_r = cv2.remap(frame_r_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

    # YOLO on both frames
    results_l = _model.predict(frame_l, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)
    results_r = _model.predict(frame_r, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)

    X = Y = Z = None
    cx = cy = None
    disparity = None
    conf = None

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
                if abs(cy - y_r) < MATCH_Y_TOL:
                    closest_box_r = box_r
                    break

        if closest_box_r is not None:
            x_r = int(closest_box_r.xywh[0][0].item())
            disparity = abs(cx - x_r)

            if disparity and disparity > 0:
                # Standard: calculate_3d_coords returns X,Y,Z in the units implied by calibration T
                R = 0
                X, Y, R = calculate_3d_coords(disparity, cx, cy, _Q)
                # Z =  math.sqrt(R**2 - X**2 - Y**2)
                Z = R

                alpha = math.atan2(X, Z)
                r = math.hypot(X, Z)
                alpha = alpha + math.radians(3.17) 
                X = math.sin(alpha) * r - 5
                Z = math.cos(alpha) * r

    t_proc = time.perf_counter() - t0
    return {
        "found": X is not None,
        "X": X, "Y": Y, "Z": Z,
        "cx": cx, "cy": cy,
        "disparity": disparity,
        "conf": conf,
        "t_proc": t_proc
    }


# ============================================================
# MOTION & AUTONOMY
# ============================================================
def drive_distance(d_cm, forward: bool):
    enc.zero()
    enc.update()
    d_cm = d_cm - 1
    if forward:
        drive_forward()
    else:
        drive_reverse()
    while abs(enc.output_revolutions()) * (math.pi * 7.2) < d_cm:
        print("Distance traveled: " + str(enc.output_revolutions() * (math.pi * 7.2)))
        time.sleep(0.01)
        enc.update()
    stop_drive()


def turn_angle(theta_rad, left_turn: bool):
    theta_rad = theta_rad - math.radians(5)
    print(theta_rad)
    enc.zero()
    enc.update()
    radius = 39.0 / 2.0
    segment = radius * abs(theta_rad)
    if left_turn:
        turn_left(0.4)
    else:
        turn_right(0.4)
    while abs(enc.output_revolutions()) * (math.pi * 7.2) < segment:
        print("Distance traveled: " + str(enc.output_revolutions() * (math.pi * 7.2)))
        time.sleep(0.01)
        enc.update()
    stop_drive()


def bring_bottle():
    time.sleep(3)
    t0 = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now - t0 > 10:
            break
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            break
        else:
            print("No bottle detected.")

    if found_bottle["Z"] > 80:
        drive_distance(found_bottle["Z"] - 35, True)
        print("Approached bottle.")

        time.sleep(2)
        t0 = time.perf_counter()
        while True:
            found_bottle = detect_bottle_once()
            now = time.perf_counter()
            if now - t0 > 10:
                break
            if found_bottle["found"]:
                print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
                break
            else:
                print("Lost bottle :(")
                drive_distance(found_bottle["Z"], True)
    drive_distance(found_bottle["Z"], True)
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_distance(original_z, False)


def bring_bottle_xz():
    global _red_led_fail_until
    print("[AUTO] Starting detection loop...")

    # ×•×™×“×•× ×¦×‘×ª ×¤×ª×•×—×” ×œ×¤× ×™ ×ª×—×™×œ×ª ×”×ª× ×•×¢×”
    servo_move_step(0)
    time.sleep(1)

    start_time = time.time()
    found_bottle = {"found": False}

    # ×œ×•×œ××ª × ×™×¡×™×•× ×•×ª ×–×™×”×•×™ ×œ×ž×©×š 5 ×©× ×™×•×ª ×œ×ž×–×¢×•×¨ ×©×’×™××•×ª ×’×™×œ×•×™ (BER) [cite: 13, 42]
    while (time.time() - start_time) < 5.0:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            break
        time.sleep(0.1)

    if found_bottle["found"]:
        # ×”×¦×œ×—×”: ×”×“×œ×§×ª ×œ×“×™× ×™×¨×•×§×™×
        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        time.sleep(1)

        # ×›×™×‘×•×™ ×œ×“×™× ×™×¨×•×§×™× ×¨×’×¢ ×œ×¤× ×™ ×ª×—×™×œ×ª ×”× ×¡×™×¢×”
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)

        # ×¨×¦×£ ×ª× ×•×¢×” ×•×ª×¤×™×¡×”
        alpha = math.atan2(found_bottle['X'], 22 + found_bottle['Z'])
        turn_angle(alpha, alpha < 0)
        drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), True)

        time.sleep(1)
        servo_move_step(1)  # ×¡×’×™×¨×”
        time.sleep(1)

        # ×”×¨×ž×ª ×–×¨×•×¢ 30 ×ž×¢×œ×•×ª (170 ×¦×¢×“×™×)
        _stepper.step_chunk(170, direction=1, delay_sec=STEPPER_STEP_DELAY_SEC)
        time.sleep(0.5)

        drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), False)

        # ×”×•×¨×“×ª ×–×¨×•×¢ ×—×–×¨×”
        _stepper.step_chunk(170, direction=-1, delay_sec=STEPPER_STEP_DELAY_SEC)
        time.sleep(0.5)

        servo_move_step(0)  # ×©×—×¨×•×¨
        time.sleep(1)
    else:
        # ×›×©×œ: ×”×’×“×¨×ª ×˜×™×™×ž×¨ ×œ×œ×“×™× ××“×•×ž×™× ×œ×ž×©×š 5 ×©× ×™×•×ª
        print("[AUTO] Timeout: Bottle not found.")
        _red_led_fail_until = time.time() + 5.0


def shutdown_vision():
    global _cap_l, _cap_r
    try:
        if _cap_l is not None:
            _cap_l.release()
        if _cap_r is not None:
            _cap_r.release()
    except Exception:
        pass


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    global _stepper_dir, _last_pkt_t, sock, last_rx, _red_led_fail_until
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.02)
    last_rx = _last_pkt_t = time.time()
    print(f"debug: Listening for UDP packets on {UDP_LISTEN_IP}:{UDP_PORT}...")
    while True:
        ultrasonic_tick()

        now = time.time()
        too_close = obstacle_too_close()
        # ×—×™×•×•×™ ××“×•× ×× ×™×© ×ž×›×©×•×œ ××• ×× ×™×© ×›×©×œ ×–×™×”×•×™ ×‘×˜×™×™×ž×¨
        fail_active = (now < _red_led_fail_until)

        if too_close or fail_active:
            GPIO.output(RED_LED_PIN, GPIO.LOW)  # ×“×•×œ×§ (PNP)
        else:
            GPIO.output(RED_LED_PIN, GPIO.HIGH)  # ×›×‘×•×™

        if now - _last_pkt_t > STEP_CMD_STALE_SEC:
            if _stepper_dir != 0: _stepper_dir = 0; _stepper.deenergize()

        if _stepper_dir != 0: stepper_tick(_stepper_dir)

        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC: stop_drive()

        time.sleep(CONTROL_PERIOD_SEC)



def handle_payload(payload: int):
    global prev_f0, prev_f1, prev_f2, prev_f3, _stepper_dir, _last_pkt_t
    global _prev_drive_req, _prev_prev_drive_req, _ignore_ultra_active, _ignore_ultra_cmd

    _last_pkt_t = time.time()

    flex = (payload >> 4) & 0x0F
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # ---- autonomy trigger (keep your existing behavior) ----
    if (f0 == 1 and f1 == 1 and f2 == 1 and f3 == 1) and not (
            prev_f0 == 1 and prev_f1 == 1 and prev_f2 == 1 and prev_f3 == 1):
        bring_bottle_xz()

    # ---- servo edge detection ----
    if f3 == 1 and prev_f3 == 0:
        servo_move_step(1)  # close
    if f2 == 1 and prev_f2 == 0:
        servo_move_step(0)  # open

    # ---- stepper held command ----
    if f0 == 1 and f1 == 0:
        _stepper_dir = -1
    elif f1 == 1 and f0 == 0:
        _stepper_dir = +1
    else:
        _stepper_dir = 0

    # ---- update finger history for edge detection ----
    prev_f0, prev_f1, prev_f2, prev_f3 = f0, f1, f2, f3

    # ---- decode drive request ----
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
    # Ignore-ultrasonic MODE logic:
    # Arm when we see: X, STOP, X   (X in {FWD, LEFT, RIGHT})
    # Keep active only while holding the same X.
    # Disable when STOP or REV or direction change.
    # ------------------------------------------------------------
    allowed_dir = {"FWD", "LEFT", "RIGHT"}

    # disable ignore mode if STOP/REV/changed dir
    if _ignore_ultra_active:
        if drive_req in ("STOP", "REV") or drive_req != _ignore_ultra_cmd:
            _ignore_ultra_active = False
            _ignore_ultra_cmd = None

    # arm ignore mode on pattern X,STOP,X
    if (not _ignore_ultra_active) and (drive_req in allowed_dir):
        if _prev_drive_req == "STOP" and _prev_prev_drive_req == drive_req:
            _ignore_ultra_active = True
            _ignore_ultra_cmd = drive_req
            print(f"[MODE] Ignoring ultrasonics while holding {drive_req}")

    ignore_ultra_now = (_ignore_ultra_active and drive_req == _ignore_ultra_cmd)

    # ---- apply drive (reverse always allowed) ----
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

    # ---- update drive history (requested commands) ----
    _prev_prev_drive_req = _prev_drive_req
    _prev_drive_req = drive_req


if __name__ == "__main__":
    try:
        init_vision();
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_drive()
        GPIO.output(RED_LED_PIN, GPIO.HIGH);
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        GPIO.cleanup();
        print("\n[OFF] System Stopped.")
