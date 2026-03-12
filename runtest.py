#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
import pigpio
import RPi.GPIO as GPIO
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
RED_LED_PIN = 26
GREEN_LED_PIN = 16

factory = PiGPIOFactory()

# ---- Stereo / YOLO config ----
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"


W = 640
H = 480
CAM_FPS = 15
DEFAULT_IMG_SIZE = (W, H)

BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50
MATCH_Y_TOL = 10

STEPPER_STEP_DELAY_SEC = 0.0015

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RED_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)

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

def drive_forward(speed: float = 0.45):
    RIGHT_RPWM.value, LEFT_RPWM.value = speed + 0.04, speed
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0

def drive_reverse(speed: float = 0.45):
    RIGHT_LPWM.value, LEFT_LPWM.value = speed + 0.04, speed
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0


def turn_right(speed: float = 0.10):
    RIGHT_LPWM.value, LEFT_RPWM.value = speed + 0.04, speed
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0
    

def turn_left(speed: float = 0.10):
    RIGHT_RPWM.value, LEFT_LPWM.value = speed + 0.04, speed
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0
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
        self.seq = [[1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]]
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
        self.deenergize(); self.pi.stop()

_stepper = StepperMotor([23, 22, 27, 17])

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
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    if _vision_ready:
        return

    print("[VISION] Initializing YOLO + stereo...", flush=True)
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')

    _cap_l = open_cam(LEFT_CAM, "LEFT")
    _cap_r = open_cam(RIGHT_CAM, "RIGHT")
    if not _cap_l.isOpened() or not _cap_r.isOpened():
        raise RuntimeError("One or both cameras failed to open.")

    _maps_l, _maps_r, _Q = (*load_and_prepare_calibration(CALIBRATION_FILE_PATH),)
    _vision_ready = True
    print("[VISION] Ready.", flush=True)

def read_latest_stereo(cap_l, cap_r, flush_n=10):
    for _ in range(flush_n):
        cap_l.grab()
        cap_r.grab()
    ret_l, frame_l = cap_l.retrieve()
    ret_r, frame_r = cap_r.retrieve()
    return ret_l, frame_l, ret_r, frame_r

def detect_bottle_once():
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    t0 = time.perf_counter()
    ret_l, frame_l_orig, ret_r, frame_r_orig = read_latest_stereo(_cap_l, _cap_r, flush_n=5)

    if not ret_l or not ret_r:
        return {"found": False, "err": "Failed to read frames", "t_proc": time.perf_counter() - t0}

    frame_l = cv2.remap(frame_l_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    frame_r = cv2.remap(frame_r_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

    results_l = _model.predict(frame_l, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)
    results_r = _model.predict(frame_r, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)

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
                if abs(cy - y_r) < MATCH_Y_TOL:
                    closest_box_r = box_r
                    break

        if closest_box_r is not None:
            x_r = int(closest_box_r.xywh[0][0].item())
            disparity = abs(cx - x_r)

            if disparity and disparity > 0:
                R = 0
                X, Y, R = calculate_3d_coords(disparity, cx, cy, _Q)
                Z = R

                alpha = math.atan2(X, Z)
                r = math.hypot(X, Z)
                alpha = alpha + math.radians(3)
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

def shutdown_vision():
    global _cap_l, _cap_r
    try:
        if _cap_l is not None: _cap_l.release()
        if _cap_r is not None: _cap_r.release()
    except Exception:
        pass

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
    time.sleep(2)
    drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), True)
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_distance(math.hypot(found_bottle["Z"], found_bottle["X"]), False)

def track_bottle_continuous():
    # Run continuous detection loop. Press Ctrl+C to exit this loop.
    print("\n[VISION] Starting continuous tracking. Press Ctrl+C to return to menu.")
    try:
        while True:
            bottle = detect_bottle_once()
            if bottle["found"]:
                print(f"Bottle Location -> X: {bottle['X']:.2f}, Y: {bottle['Y']:.2f}, Z: {bottle['Z']:.2f}")
            else:
                print("Bottle not found...")
            
            # Small delay to stabilize loop and reduce console spam
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[VISION] Tracking stopped. Returning to menu.")

# ============================================================
# MAIN EXECUTION
# ============================================================
if __name__ == "__main__":
    try:
        init_vision()
        print("\n--- SYSTEM READY ---")
        print("Type 'c' and press Enter to start bringing the bottle.")
        print("Type 'q' and press Enter to quit.\n")
        
        while True:
            cmd = input("Command (c/q): ").strip().lower()
            if cmd == 'c':
                bring_bottle_xz()
            elif cmd == 'b':
                track_bottle_continuous()
            elif cmd == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        stop_drive()
        GPIO.output(RED_LED_PIN, GPIO.HIGH)
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        shutdown_vision()
        GPIO.cleanup()
        print("[OFF] System Stopped.")