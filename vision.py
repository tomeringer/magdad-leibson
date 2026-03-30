import os, cv2, pickle, math, time
import numpy as np
from ultralytics import YOLO

BOTTLE_CLASS_ID = 39
YOLO_CONF_THRESHOLD = 0.50
MATCH_Y_TOLERANCE = 10
CAMERA_YAW_OFFSET_RAD = math.radians(3)
CAMERA_X_OFFSET_CM = 5.0

_model = _cap_l = _cap_r = _maps_l = _maps_r = _Q = None

def init(left_path, right_path, calib_path):
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null")
    time.sleep(2); os.system("echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null")
    time.sleep(4)
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')
    _cap_l = cv2.VideoCapture(left_path, cv2.CAP_V4L2)
    _cap_r = cv2.VideoCapture(right_path, cv2.CAP_V4L2)
    for c in [_cap_l, _cap_r]:
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 640); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        c.set(cv2.CAP_PROP_FPS, 5); c.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    with open(calib_path, 'rb') as f:
        data = pickle.load(f)
    _Q = data['Q']
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(data['cameraMatrix1'], data['distCoeffs1'], data['cameraMatrix2'], data['distCoeffs2'], (640,480), data['R'], data['T'], alpha=-1)
    _maps_l = cv2.initUndistortRectifyMap(data['cameraMatrix1'], data['distCoeffs1'], R1, P1, (640,480), cv2.CV_32FC1)
    _maps_r = cv2.initUndistortRectifyMap(data['cameraMatrix2'], data['distCoeffs2'], R2, P2, (640,480), cv2.CV_32FC1)

def detect_bottle_once():
    for _ in range(5): _cap_l.grab(); _cap_r.grab()
    ret_l, fl = _cap_l.retrieve(); ret_r, fr = _cap_r.retrieve()
    if not ret_l or not ret_r: return {"found": False}
    fl = cv2.remap(fl, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    fr = cv2.remap(fr, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)
    res_l = _model.predict(fl, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)
    res_r = _model.predict(fr, conf=YOLO_CONF_THRESHOLD, classes=[BOTTLE_CLASS_ID], verbose=False)
    X = Y = Z = None
    if res_l and len(res_l[0].boxes) > 0:
        box_l = res_l[0].boxes[0]
        cx, cy = int(box_l.xywh[0][0]), int(box_l.xywh[0][1])
        if res_r and len(res_r[0].boxes) > 0:
            for box_r in res_r[0].boxes:
                xr, yr = int(box_r.xywh[0][0]), int(box_r.xywh[0][1])
                if abs(cy - yr) < MATCH_Y_TOLERANCE:
                    disparity = abs(cx - xr)
                    if disparity > 0:
                        p = np.dot(_Q, np.array([cx, cy, disparity, 1], dtype=np.float32))
                        X_raw, Y, Z_raw = p[0]/p[3], p[1]/p[3], p[2]/p[3]
                        alpha = math.atan2(X_raw, Z_raw) + CAMERA_YAW_OFFSET_RAD
                        r = math.hypot(X_raw, Z_raw)
                        X, Z = math.sin(alpha)*r - CAMERA_X_OFFSET_CM, math.cos(alpha)*r
                        break
    return {"found": X is not None, "X": X, "Z": Z}

def shutdown():
    if _cap_l: _cap_l.release()
    if _cap_r: _cap_r.release()