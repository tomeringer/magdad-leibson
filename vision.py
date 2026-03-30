import cv2
import math
import os
import pickle
import time

import numpy as np
from ultralytics import YOLO

# Constants
BOTTLE_CLASS_ID = 39
YOLO_CONF_THRESHOLD = 0.50
MATCH_Y_TOLERANCE = 10
CAMERA_YAW_OFFSET_RAD = math.radians(3)
CAMERA_X_OFFSET_CM = 5.0
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 5

# Module State
_model = _cap_l = _cap_r = _maps_l = _maps_r = _Q = None


def _open_cam(path, name):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened(): return cap
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10): cap.read()
    return cap


def init_vision(left_path, right_path, calib_path):
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null")
    time.sleep(2)
    os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null")
    time.sleep(4)

    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')
    _cap_l = _open_cam(left_path, "LEFT")
    _cap_r = _open_cam(right_path, "RIGHT")

    with open(calib_path, 'rb') as f:
        data = pickle.load(f)
    K1, D1, K2, D2, R, T, _Q = data['cameraMatrix1'], data['distCoeffs1'], data['cameraMatrix2'], data['distCoeffs2'], \
        data['R'], data['T'], data['Q']

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, (CAM_WIDTH, CAM_HEIGHT), R, T, alpha=-1)
    _maps_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (CAM_WIDTH, CAM_HEIGHT), cv2.CV_32FC1)
    _maps_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (CAM_WIDTH, CAM_HEIGHT), cv2.CV_32FC1)


def calculate_3d_coords(disparity, x, y, Q_matrix):
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    hp = np.dot(Q_matrix, point)
    w = hp[3]
    if w == 0: return (0, 0, 0)
    return (hp[0] / w, hp[1] / w, hp[2] / w)


def detect_bottle_once():
    for _ in range(5): _cap_l.grab(); _cap_r.grab()
    ret_l, fl_orig = _cap_l.retrieve();
    ret_r, fr_orig = _cap_r.retrieve()
    if not ret_l or not ret_r: return {"found": False}

    fl = cv2.remap(fl_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    fr = cv2.remap(fr_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

    res_l = _model.predict(fl, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)
    res_r = _model.predict(fr, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)

    X = Y = Z = None
    if res_l and len(res_l[0].boxes) > 0:
        box_l = res_l[0].boxes[0]
        cx, cy = int(box_l.xywh[0][0]), int(box_l.xywh[0][1])

        closest_r = None
        if res_r and len(res_r[0].boxes) > 0:
            for box_r in res_r[0].boxes:
                xr, yr = int(box_r.xywh[0][0]), int(box_r.xywh[0][1])
                if abs(cy - yr) < MATCH_Y_TOLERANCE:
                    closest_r = box_r;
                    break

        if closest_r is not None:
            disparity = abs(cx - int(closest_r.xywh[0][0]))
            if disparity > 0:
                X_raw, Y, Z_raw = calculate_3d_coords(disparity, cx, cy, _Q)
                alpha = math.atan2(X_raw, Z_raw) + CAMERA_YAW_OFFSET_RAD
                r = math.hypot(X_raw, Z_raw)
                X = math.sin(alpha) * r - CAMERA_X_OFFSET_CM
                Z = math.cos(alpha) * r

    return {"found": X is not None, "X": X, "Y": Y, "Z": Z}


def shutdown_vision():
    if _cap_l: _cap_l.release()
    if _cap_r: _cap_r.release()
