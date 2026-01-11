<<<<<<< HEAD
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
from datetime import datetime
import math
import pickle
from typing import Optional

import cv2
import numpy as np
from ultralytics import YOLO

import pigpio
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory


# ============================================================
# CONFIGURATION
# ============================================================
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

# ============================================================
# ULTRASONIC SENSOR
# ============================================================
TRIG, ECHO = 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ULTRA_STOP_CM = 40.0
_last_distance_cm: Optional[float] = None


def measure_distance_cm() -> Optional[float]:
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > 0.03:
            return None
    ps = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - ps > 0.03:
            return None
    pe = time.time()
    return (pe - ps) * 17150.0


# ============================================================
# SERVO - Step Logic with Clamping
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo = None


def servo_init():
    global _servo
    _servo = Servo(
        SERVO_PIN,
        initial_value=None,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000
    )
    print(f"[SERVO] Ready on GPIO {SERVO_PIN}", flush=True)


def servo_move_step(direction: int):
    """
    direction: 1 for minus step, 0 for plus step
    """
    global current_pos
    if _servo is None:
        return

    new_val = current_pos - MOVE_STEP if direction == 1 else current_pos + MOVE_STEP
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos
    print(f"[SERVO] Moved to {current_pos:.2f}", flush=True)


# ============================================================
# DC & STEPPER MOTORS
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


def turn_right():
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0


def turn_left():
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0


class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [[1, 0, 1, 0],
                    [0, 1, 1, 0],
                    [0, 1, 0, 1],
                    [0, 0, 1, 1]]
        for p in self.pins:
            self.pi.set_mode(p, pigpio.OUTPUT)

    def move(self, steps, direction=1):
        for _ in range(int(steps)):
            for step in range(len(self.seq)):
                idx = step if direction == 1 else (3 - step)
                for i, p in enumerate(self.pins):
                    self.pi.write(p, self.seq[idx][i])
                time.sleep(0.005)
        for p in self.pins:
            self.pi.write(p, 0)


_stepper = StepperMotor([23, 22, 27, 17])


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


def detect_bottle_once():
    """
    Grabs one stereo pair and tries to compute bottle (X,Y,Z) and pixel center.
    Returns dict with keys: found, X,Y,Z,cx,cy,disparity,conf,t_proc
    """
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q

    t0 = time.perf_counter()

    ret_l, frame_l_orig = _cap_l.read()
    ret_r, frame_r_orig = _cap_r.read()
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
                
    t_proc = time.perf_counter() - t0
    return {
        "found": X is not None,
        "X": X, "Y": Y, "Z": Z,
        "cx": cx, "cy": cy,
        "disparity": disparity,
        "conf": conf,
        "t_proc": t_proc
    }


def bring_bottle():
    V = 96  # Forward speed cm/s
    while True:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            break
        else:
            print("No bottle detected.")
    
    servo_move_step(1)
    drive_forward()
    time.sleep(found_bottle["Z"]/(V*0.5))  
    stop_drive()
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_reverse()
    time.sleep(original_z/(V*0.5))
    stop_drive()



def bring_bottle_advanced():
    V = 96  # Forward speed cm/s
    while True:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            break
        else:
            print("No bottle detected.")

    drive_forward(0.2)
    # start_time = datetime.now().time()
    # start_time_seconds = start_time.hour*3600+start_time.minute*60+start_time.second
    while found_bottle["Z"] > 30.0:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}, Time={datetime.now().time()}")
        else:
            print("Lost bottle during approach.")
            stop_drive()
            return
    # Arrived 10cm from bottle bottle
    stop_drive()
    found_bottle = detect_bottle_once()
    servo_move_step(1)  
    drive_forward(0.2)
    time.sleep(found_bottle["Z"]/(V*0.2))  
    stop_drive()
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_reverse()
    time.sleep(original_z/(V*0.5))
    stop_drive()


def drive_2_seconds_forward():
    drive_forward()
    time.sleep(4)
    stop_drive()


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
# KEYBOARD LOOP
# ============================================================
def run_keyboard_loop():
    global _last_distance_cm

    print("\n[READY] w/s/a/d=Drive, x=Stop, i/k=Servo, u/j=Stepper, c=Detect bottle, q=Quit", flush=True)

    while True:
        try:
            cmd = input("cmd> ").strip().lower()
            if not cmd:
                continue
            c = cmd[0]

            if c == 'q':
                print("[QUIT]", flush=True)
                break

            # Optionally show distance for each command (useful safety feedback)
            _last_distance_cm = measure_distance_cm()
            too_close = (_last_distance_cm is not None and _last_distance_cm < ULTRA_STOP_CM)
            if _last_distance_cm is None:
                print("[ULTRA] distance=None (timeout)", flush=True)
            else:
                print(f"[ULTRA] distance={_last_distance_cm:.1f} cm (stop<{ULTRA_STOP_CM:.1f})", flush=True)

            if c == 'x':
                stop_drive()
                print("[STOP] Drive stopped", flush=True)

            elif c == 'w':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    drive_forward()
                    print("[DRIVE] Forward", flush=True)

            elif c == 's':
                drive_reverse()
                print("[DRIVE] Reverse", flush=True)

            elif c == 'a':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    turn_left()
                    print("[TURN] Left", flush=True)

            elif c == 'd':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    turn_right()
                    print("[TURN] Right", flush=True)

            elif c == 'i':
                print("[SERVO] Close step", flush=True)
                servo_move_step(1)

            elif c == 'k':
                print("[SERVO] Open step", flush=True)
                servo_move_step(0)

            elif c == 'u':
                print("[STEPPER] +50", flush=True)
                _stepper.move(50, 1)

            elif c == 'j':
                print("[STEPPER] -50", flush=True)
                _stepper.move(50, -1)

            elif c == 'c':
                print("[VISION] Detecting bottle (one-shot)...", flush=True)
                res = detect_bottle_once()
                print("[VISION] result:", res, flush=True)

            elif c == 'b':
                print("[VISION] Starting continuous bottle detection. Ctrl-C to stop.", flush=True)
                bring_bottle()
                print("[VISION] Finished bring_bottle.", flush=True)
            
            elif c == 'f':
                print("[DRIVE] Driving forward for 2 seconds.", flush=True)
                drive_2_seconds_forward()

            else:
                print("[INFO] Unknown command. Use w/s/a/d/x/i/k/u/j/c/q", flush=True)

        except UnicodeDecodeError:
            print("[ERROR] Switch keyboard to ENGLISH!", flush=True)
        except KeyboardInterrupt:
            print("\n[QUIT] KeyboardInterrupt", flush=True)
            break
        except Exception as e:
            print(f"[ERROR] {e}", flush=True)

    stop_drive()


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    servo_init()
    # optional: init vision at startup so first 'c' is fast
    # init_vision()

    try:
        init_vision()
        run_keyboard_loop()
    finally:
        stop_drive()
        shutdown_vision()
        GPIO.cleanup()
        print("\n[OFF] System Stopped.", flush=True)
=======
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
from datetime import datetime
import math
import pickle
from typing import Optional

import cv2
import numpy as np
from ultralytics import YOLO

import pigpio
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory


# ============================================================
# CONFIGURATION
# ============================================================
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

# ============================================================
# ULTRASONIC SENSOR
# ============================================================
TRIG, ECHO = 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ULTRA_STOP_CM = 40.0
_last_distance_cm: Optional[float] = None


def measure_distance_cm() -> Optional[float]:
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > 0.03:
            return None
    ps = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - ps > 0.03:
            return None
    pe = time.time()
    return (pe - ps) * 17150.0


# ============================================================
# SERVO - Step Logic with Clamping
# ============================================================
SERVO_PIN = 12
MOVE_STEP = 0.39
current_pos = 0.0
_servo = None


def servo_init():
    global _servo
    _servo = Servo(
        SERVO_PIN,
        initial_value=None,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000
    )
    print(f"[SERVO] Ready on GPIO {SERVO_PIN}", flush=True)


def servo_move_step(direction: int):
    """
    direction: 1 for minus step, 0 for plus step
    """
    global current_pos
    if _servo is None:
        return

    new_val = current_pos - MOVE_STEP if direction == 1 else current_pos + MOVE_STEP
    current_pos = max(-1.0, min(1.0, new_val))
    _servo.value = current_pos
    print(f"[SERVO] Moved to {current_pos:.2f}", flush=True)


# ============================================================
# DC & STEPPER MOTORS
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


def turn_right():
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.51, 0.50
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0


def turn_left():
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.51, 0.50
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0


class StepperMotor:
    def __init__(self, pins):
        self.pins = pins
        self.pi = pigpio.pi()
        self.seq = [[1, 0, 1, 0],
                    [0, 1, 1, 0],
                    [0, 1, 0, 1],
                    [0, 0, 1, 1]]
        for p in self.pins:
            self.pi.set_mode(p, pigpio.OUTPUT)

    def move(self, steps, direction=1):
        for _ in range(int(steps)):
            for step in range(len(self.seq)):
                idx = step if direction == 1 else (3 - step)
                for i, p in enumerate(self.pins):
                    self.pi.write(p, self.seq[idx][i])
                time.sleep(0.005)
        for p in self.pins:
            self.pi.write(p, 0)


_stepper = StepperMotor([23, 22, 27, 17])


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


def detect_bottle_once():
    """
    Grabs one stereo pair and tries to compute bottle (X,Y,Z) and pixel center.
    Returns dict with keys: found, X,Y,Z,cx,cy,disparity,conf,t_proc
    """
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q

    t0 = time.perf_counter()

    ret_l, frame_l_orig = _cap_l.read()
    ret_r, frame_r_orig = _cap_r.read()
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
                Z =  math.sqrt(R**2 - X**2 - Y**2) * 0.94 # cos(tetha) correction factor

    t_proc = time.perf_counter() - t0
    return {
        "found": X is not None,
        "X": X, "Y": Y, "Z": Z,
        "cx": cx, "cy": cy,
        "disparity": disparity,
        "conf": conf,
        "t_proc": t_proc
    }


def bring_bottle():
    V = 96  # Forward speed cm/s
    while True:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            break
        else:
            print("No bottle detected.")
    
    servo_move_step(1)
    drive_forward()
    time.sleep(found_bottle["Z"]/(V*0.5))  
    stop_drive()
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_reverse()
    time.sleep(original_z/(V*0.5))
    stop_drive()



def bring_bottle_advanced():
    V = 96  # Forward speed cm/s
    while True:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}")
            original_z = found_bottle['Z']
            break
        else:
            print("No bottle detected.")

    drive_forward(0.2)
    # start_time = datetime.now().time()
    # start_time_seconds = start_time.hour*3600+start_time.minute*60+start_time.second
    while found_bottle["Z"] > 30.0:
        found_bottle = detect_bottle_once()
        if found_bottle["found"]:
            print(f"Bottle at X={found_bottle['X']:.2f}, Y={found_bottle['Y']:.2f}, Z={found_bottle['Z']:.2f}, Time={datetime.now().time()}")
        else:
            print("Lost bottle during approach.")
            stop_drive()
            return
    # Arrived 10cm from bottle bottle
    stop_drive()
    found_bottle = detect_bottle_once()
    servo_move_step(1)  
    drive_forward(0.2)
    time.sleep(found_bottle["Z"]/(V*0.2))  
    stop_drive()
    print("Arrived at bottle location.")
    time.sleep(1)
    servo_move_step(0)
    time.sleep(1)
    drive_reverse()
    time.sleep(original_z/(V*0.5))
    stop_drive()


def drive_2_seconds_forward():
    drive_forward()
    time.sleep(4)
    stop_drive()


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
# KEYBOARD LOOP
# ============================================================
def run_keyboard_loop():
    global _last_distance_cm

    print("\n[READY] w/s/a/d=Drive, x=Stop, i/k=Servo, u/j=Stepper, c=Detect bottle, q=Quit", flush=True)

    while True:
        try:
            cmd = input("cmd> ").strip().lower()
            if not cmd:
                continue
            c = cmd[0]

            if c == 'q':
                print("[QUIT]", flush=True)
                break

            # Optionally show distance for each command (useful safety feedback)
            _last_distance_cm = measure_distance_cm()
            too_close = (_last_distance_cm is not None and _last_distance_cm < ULTRA_STOP_CM)
            if _last_distance_cm is None:
                print("[ULTRA] distance=None (timeout)", flush=True)
            else:
                print(f"[ULTRA] distance={_last_distance_cm:.1f} cm (stop<{ULTRA_STOP_CM:.1f})", flush=True)

            if c == 'x':
                stop_drive()
                print("[STOP] Drive stopped", flush=True)

            elif c == 'w':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    drive_forward()
                    print("[DRIVE] Forward", flush=True)

            elif c == 's':
                drive_reverse()
                print("[DRIVE] Reverse", flush=True)

            elif c == 'a':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    turn_left()
                    print("[TURN] Left", flush=True)

            elif c == 'd':
                if too_close:
                    stop_drive()
                    print("[BLOCKED] Too close -> stop", flush=True)
                else:
                    turn_right()
                    print("[TURN] Right", flush=True)

            elif c == 'i':
                print("[SERVO] Close step", flush=True)
                servo_move_step(1)

            elif c == 'k':
                print("[SERVO] Open step", flush=True)
                servo_move_step(0)

            elif c == 'u':
                print("[STEPPER] +50", flush=True)
                _stepper.move(50, 1)

            elif c == 'j':
                print("[STEPPER] -50", flush=True)
                _stepper.move(50, -1)

            elif c == 'c':
                print("[VISION] Detecting bottle (one-shot)...", flush=True)
                res = detect_bottle_once()
                print("[VISION] result:", res, flush=True)

            elif c == 'b':
                print("[VISION] Starting continuous bottle detection. Ctrl-C to stop.", flush=True)
                bring_bottle()
                print("[VISION] Finished bring_bottle.", flush=True)
            
            elif c == 'f':
                print("[DRIVE] Driving forward for 2 seconds.", flush=True)
                drive_2_seconds_forward()

            else:
                print("[INFO] Unknown command. Use w/s/a/d/x/i/k/u/j/c/q", flush=True)

        except UnicodeDecodeError:
            print("[ERROR] Switch keyboard to ENGLISH!", flush=True)
        except KeyboardInterrupt:
            print("\n[QUIT] KeyboardInterrupt", flush=True)
            break
        except Exception as e:
            print(f"[ERROR] {e}", flush=True)

    stop_drive()


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    servo_init()
    # optional: init vision at startup so first 'c' is fast
    # init_vision()

    try:
        init_vision()
        run_keyboard_loop()
    finally:
        stop_drive()
        shutdown_vision()
        GPIO.cleanup()
        print("\n[OFF] System Stopped.", flush=True)
>>>>>>> e458e0d39b8fbfbfeb3d8d0900d6fed0d3cdba0e
