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
MODE = "GLOVE"
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210
FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7

factory = PiGPIOFactory()

# ---- Stereo / YOLO config ----

CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"

LEFT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"

RIGHT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

W = 640
H = 480
CAM_FPS = 15
DEFAULT_IMG_SIZE = (W, H)

# COCO bottle class
BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50

# Epipolar match tolerance in rectified images

MATCH_Y_TOL = 10

# ---- Stepper "continuous feel" ----
STEPPER_CHUNK = 4                 # keep small so stop feels immediate
STEPPER_STEP_DELAY_SEC = 0.0015   # was 0.0015
STEPPER_TICK_SEC = 0.05           # glove sends every 50ms -> match it
_last_stepper_cmd_t = 0.0

# ---- Stepper command freshness gate (Choice #1) ----
STEP_CMD_STALE_SEC = 0.12         # if no valid packets for this long -> stop stepper immediately
_stepper_dir = 0                  # -1, 0, +1 (desired direction while held)
_last_pkt_t = 0.0                 # last time we received a valid framed packet
sock = None
last_rx = 0.0

# ============================================================
# GLOBAL STATE (edge-detected)
# ============================================================
prev_f0 = prev_f1 = prev_f2 = prev_f3 = 0
# ============================================================
# DRIVE HISTORY (for ultrasonic override pattern)
#   Pattern to override ultrasonic "too close":
#     <DIR>, STOP, <DIR>   where DIR in {FWD, LEFT, RIGHT}
#   Once triggered, override stays active while holding the same DIR,
#   and turns off when you STOP, REVERSE, or change direction.
# ============================================================
_prev_drive_req = "STOP"
_prev_prev_drive_req = "STOP"

# Ignore-ultrasonic mode: armed by X,0,X. Disabled by STOP(0) or REV.
_ignore_ultra_active = False
_ignore_ultra_cmd = None  # "FWD"/"LEFT"/"RIGHT"



# ============================================================
# ULTRASONIC SENSORS (ASYNC)
# ============================================================
US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(US1_TRIG, GPIO.OUT)
GPIO.setup(US1_ECHO, GPIO.IN)
GPIO.setup(US2_TRIG, GPIO.OUT)
GPIO.setup(US2_ECHO, GPIO.IN)

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
        if time.time() - t0 > US_TIMEOUT_SEC:
            return None

    ps = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - ps > US_TIMEOUT_SEC:
            return None

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



# --- הגדרות חומרה ---
SERVO_PIN = 12
factory = PiGPIOFactory()

# הפתרון כאן: initial_value=None גורם למנוע לא לזוז בכלל כשהקוד עולה
servo = Servo(
    SERVO_PIN,
    initial_value=None,
    min_pulse_width=0.5 / 1000,
    max_pulse_width=2.5 / 1000,
    pin_factory=factory
)

# --- הגדרות תנועה ---
MOVE_STEP = 0.39  # 70 מעלות עבור מנוע 360

# אנחנו מתחילים ב-0, אבל המנוע לא ידע מזה עד הלחיצה הראשונה
current_pos = 0.0
# Servo logical state (start OPEN)
servo_is_open = True


def servo_move_step(direction):
    global current_pos, servo_is_open

    # direction: 1 = close, 0 = open
    want_close = (direction == 1)

    # Ignore redundant commands
    if want_close and (not servo_is_open):
        print(">> Servo already CLOSED - ignoring close command")
        return
    if (not want_close) and servo_is_open:
        print(">> Servo already OPEN - ignoring open command")
        return

    #new_val = current_pos
    # Apply one step
    if want_close:
        new_val = current_pos + MOVE_STEP
    else:
         new_val = current_pos - MOVE_STEP

    # Clamp
    if new_val > 1.0: new_val = 1.0
    if new_val < -1.0: new_val = -1.0

    current_pos = new_val
    servo.value = current_pos

    # Update logical state only if we actually moved
    servo_is_open = (not want_close)

    state_str = "OPEN" if servo_is_open else "CLOSED"
    print(f">> Servo moved to {current_pos:.2f} | state={state_str}")


# ============================================================
# DC MOTORS
# ============================================================
RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM  = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM  = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)


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
# ENCODER
# ============================================================
class YellowJacketEncoder:
    """
    goBILDA Yellow Jacket encoder
    Spec used directly:
        384.5 PPR at OUTPUT SHAFT
        Quadrature decoding -> x4
    """

    # Quadrature transition table
    _TRANS = {
        0b0001: +1, 0b0010: -1,
        0b0100: -1, 0b0111: +1,
        0b1000: +1, 0b1011: -1,
        0b1101: -1, 0b1110: +1,
    }

    def __init__(self, pi, gpio_a, gpio_b):
        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b

        # ===== SITE SPEC (USED DIRECTLY) =====
        self.ppr_output = 384.5 * (100 / 106)
        self.counts_per_rev_output = self.ppr_output  # 1538 counts/rev

        self.counts = 0
        self._last_state = 0
        self._t_last = time.time()
        self._c_last = 0
        self._output_rpm = 0.0

        for g in (gpio_a, gpio_b):
            self.pi.set_mode(g, pigpio.INPUT)
            self.pi.set_pull_up_down(g, pigpio.PUD_UP)

        a = self.pi.read(gpio_a)
        b = self.pi.read(gpio_b)
        self._last_state = (a << 1) | b

        self._cba = self.pi.callback(gpio_a, pigpio.EITHER_EDGE, self._cb)
        self._cbb = self.pi.callback(gpio_b, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        a = self.pi.read(self.gpio_a)
        b = self.pi.read(self.gpio_b)
        new_state = (a << 1) | b
        key = (self._last_state << 2) | new_state
        self.counts += self._TRANS.get(key, 0)
        self._last_state = new_state

    def update(self, window_s=0.2):
        t = time.time()
        dt = t - self._t_last
        if dt < window_s:
            return

        dc = self.counts - self._c_last
        rps = (dc / self.counts_per_rev_output) / dt
        self._output_rpm = 60.0 * rps

        self._t_last = t
        self._c_last = self.counts

    # ===== Public API =====
    def output_revolutions(self):
        return self.counts / self.counts_per_rev_output

    def output_rpm(self):
        return self._output_rpm

    def output_degrees(self):
        return self.output_revolutions() * 360.0

    def zero(self):
        self.counts = 0
        self._c_last = 0
        self._t_last = time.time()
        self._output_rpm = 0.0

    def stop(self):
        self._cba.cancel()
        self._cbb.cancel()


ENC_A = 24  # free
ENC_B = 25  # free
pi = pigpio.pi()
enc = YellowJacketEncoder(pi, ENC_A, ENC_B)



# ============================================================
# STEPPER MOTOR
# ============================================================
class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
        ]
        for p in self.pins:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.write(p, 0)

    def step_chunk(self, steps: int, direction: int = 1, delay_sec: float = 0.002):
        steps = int(max(0, steps))
        direction = 1 if direction >= 0 else -1

        for _ in range(steps):
            for step in range(4):
                idx = step if direction == 1 else 3 - step
                for i, p in enumerate(self.pins):
                    self.pi.write(p, self.seq[idx][i])
                time.sleep(delay_sec)

        # de-energize between chunks
        self.deenergize()

    def deenergize(self):
        for p in self.pins:
            self.pi.write(p, 0)

    def close(self):
        self.deenergize()
        try:
            self.pi.stop()
        except Exception:
            pass


_stepper = StepperMotor([23, 22, 27, 17])


def stepper_tick(direction: int):
    global _last_stepper_cmd_t
    now = time.time()
    if now - _last_stepper_cmd_t >= STEPPER_TICK_SEC:
        _last_stepper_cmd_t = now
        _stepper.step_chunk(STEPPER_CHUNK, direction=direction, delay_sec=STEPPER_STEP_DELAY_SEC)



# ============================================================
# VISION (Stereo + YOLO) as functions
# ============================================================
# Global cached vision state so pressing 'c' is fast
_vision_ready = False
_model = None
_cap_l = None
_cap_r = None
_maps_l = None
_maps_r = None
_Q = None


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
    X = hp[0] / w
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
                alpha = alpha + math.radians(3)
                X = math.sin(alpha) * r
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
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_distance(original_z, False)


def bring_bottle_xz():
    time.sleep(2)
    z0 = 22
    t0 = time.perf_counter()
    while True:
        found_bottle = detect_bottle_once()
        now = time.perf_counter()
        if now - t0 > 10:
            break
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            new_z = z0 + original_z
            alpha = math.atan2(found_bottle['X'], new_z)
            break
        else:
            print("No bottle detected.")

    turn_angle(alpha, alpha < 0)
    print("Completed turn towards bottle.")
    drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), True)
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), False)


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
# PAYLOAD HANDLER (MATCHES YOUR ESP32 PACKET ENCODING)
#   dataByte = (flexBits<<4) | (rollBits<<2) | pitchBits
#
#   flexPins order on ESP32: {A0, A2, A1, A3}
#   so:
#     f0 -> bit0 -> A0
#     f1 -> bit1 -> A2
#     f2 -> bit2 -> A1
#     f3 -> bit3 -> A3
# ============================================================
def handle_payload(payload: int):
    global prev_f0, prev_f1, prev_f2, prev_f3
    global _stepper_dir, _last_pkt_t
    global _prev_drive_req, _prev_prev_drive_req, _ignore_ultra_active, _ignore_ultra_cmd

    _last_pkt_t = time.time()  # refresh stepper-command validity on every valid packet

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # ------------------------------------------------------------
    # Special combo: all fingers pressed -> bring bottle
    # ------------------------------------------------------------
    if (f0 == 1 and f1 == 1 and f2 == 1 and f3 == 1) and not (prev_f0 == 1 and prev_f1 == 1 and prev_f2 == 1 and prev_f3 == 1):
        bring_bottle_xz()
        prev_f0, prev_f1, prev_f2, prev_f3 = f0, f1, f2, f3
        return

    # ------------------------------------------------------------
    # Servo edge detection (now: f3=close, f2=open)
    # ------------------------------------------------------------
    if f3 == 1 and prev_f3 == 0:
        print("[EVENT] Finger 3 (A3) pressed -> Closing Servo")
        servo_move_step(1)

    if f2 == 1 and prev_f2 == 0:
        print("[EVENT] Finger 2 (A1) pressed -> Opening Servo")
        servo_move_step(0)

    # ------------------------------------------------------------
    # Stepper "held" command (now: f0=down, f1=up)
    #   Movement happens in main loop based on _stepper_dir
    # ------------------------------------------------------------
    if f0 == 1 and f1 == 0:
        _stepper_dir = -1   # down
    elif f1 == 1 and f0 == 0:
        _stepper_dir = +1   # up
    else:
        _stepper_dir = 0

    # update previous states for edge detection
    prev_f0, prev_f1, prev_f2, prev_f3 = f0, f1, f2, f3

    # ------------------------------------------------------------
    # Decide requested DRIVE command from roll/pitch
    #   (independent of ultrasonic gating)
    # ------------------------------------------------------------
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

    too_close = obstacle_too_close()

     # ------------------------------------------------------------
    # Ignore-ultrasonic MODE logic:
    # Arm when we see: X, STOP, X   (X in {FWD, LEFT, RIGHT})
    # Keep active only while holding the same X.
    # Disable when STOP (zeros) or REV (backwards) or direction change.
    # ------------------------------------------------------------
    allowed_dir = {"FWD", "LEFT", "RIGHT"}

    # Disable ignore-mode if user stops, reverses, or changes direction
    if _ignore_ultra_active:
        if drive_req in ("STOP", "REV") or drive_req != _ignore_ultra_cmd:
            _ignore_ultra_active = False
            _ignore_ultra_cmd = None

    # Arm ignore-mode on pattern: (two steps ago == current) and (previous == STOP)
    if (not _ignore_ultra_active) and (drive_req in allowed_dir):
        if _prev_drive_req == "STOP" and _prev_prev_drive_req == drive_req:
            _ignore_ultra_active = True
            _ignore_ultra_cmd = drive_req
            print(f"[MODE] Ignoring ultrasonics while holding {drive_req}")

    # While mode is active AND we keep holding the same X, ignore ultrasonic stop
    ignore_ultra_now = (_ignore_ultra_active and drive_req == _ignore_ultra_cmd)

    # ------------------------------------------------------------
    # Apply DC motors (reverse always allowed)
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # Update history of REQUESTED drive commands (not what actually happened)
    # ------------------------------------------------------------
    _prev_prev_drive_req = _prev_drive_req
    _prev_drive_req = drive_req


# ============================================================
# MAIN LOOP
# ============================================================
def run_glove_loop():
    global _stepper_dir, _last_pkt_t, sock, last_rx

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(0.02)  # faster silence detection

    print(f"[RUNNING] Glove UDP + Async Ultrasonic on Port {UDP_PORT}")

    last_rx = time.time()
    _last_pkt_t = time.time()  # initialize to "now" so we don't instantly stale-stop on startup

    while True:
        ultrasonic_tick()

        # ----- Stepper freshness gate: if packets stop, stop stepper immediately -----
        now = time.time()
        if now - _last_pkt_t > STEP_CMD_STALE_SEC:
            if _stepper_dir != 0:
                _stepper_dir = 0
                _stepper.deenergize()  # immediate stop (drop coils)

        # ----- Stepper continuous motion only while command is fresh -----
        if _stepper_dir != 0:
            stepper_tick(_stepper_dir)

        # ----- UDP receive -----
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            # Stop DC motors on long silence
            if time.time() - last_rx > SILENCE_STOP_SEC:
                stop_drive()

        time.sleep(CONTROL_PERIOD_SEC)


# ============================================================
# PIN MAPPING (Raspberry Pi, BCM numbering)
#
#   Usage                  BCM   Physical pin
#   ------------------------------------------------
#   Ultrasonic #1 TRIG      5    pin 29
#   Ultrasonic #1 ECHO      6    pin 31
#   Ultrasonic #2 TRIG     13    pin 33
#   Ultrasonic #2 ECHO     19    pin 35
#
#   Servo signal           12    pin 32
#
#   Right DC motor RPWM     2    pin 3
#   Right DC motor LPWM     3    pin 5
#   Left  DC motor RPWM    20    pin 38
#   Left  DC motor LPWM    21    pin 40
#
#   Stepper IN1            23    pin 16
#   Stepper IN2            22    pin 15
#   Stepper IN3            27    pin 13
#   Stepper IN4            17    pin 11
# ============================================================


# ============================================================
# ENTRY
# ============================================================
if __name__ == "__main__":
    try:
        init_vision()
        run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_drive()
        shutdown_vision()
        try:
            _stepper.close()
        except Exception:
            pass
        GPIO.cleanup()
        print("\n[OFF] System Stopped.", flush=True)
