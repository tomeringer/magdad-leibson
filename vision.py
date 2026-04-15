import os
import time
import math
import pickle
import cv2
import numpy as np
from ultralytics import YOLO

# ============================================================
# CONSTANTS (Required by the functions below)
# ============================================================
CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FPS = 5
DEFAULT_IMG_SIZE = (CAM_WIDTH, CAM_HEIGHT)

BOTTLE_CLASS_ID = 39
YOLO_CONF_THRESHOLD = 0.50
MATCH_Y_TOLERANCE = 10

CAMERA_YAW_OFFSET_RAD = math.radians(3)
CAMERA_X_OFFSET_CM = 5.0

# These paths usually come from main.py, defined here for safety
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
CALIBRATION_FILE_PATH = "Autonomy/stereo_calibration.pkl"

# ============================================================
# VISION STATE
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

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # for _ in range(10):
    #     cap.read()

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


def init() -> None:
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    if _vision_ready:
        return

    hard_reset_usb_all()

    print("[VISION] Initializing YOLO + stereo...", flush=True)
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')

    _cap_l = open_cam(LEFT_CAM_PATH, "LEFT")
    time.sleep(2)
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


def shutdown() -> None:
    global _vision_ready, _cap_l, _cap_r
    
    # Release hardware and reset the variables to None
    if _cap_l is not None: 
        _cap_l.release()
        _cap_l = None
        
    if _cap_r is not None: 
        _cap_r.release()
        _cap_r = None
        
    cv2.destroyAllWindows()
    
    # Crucial: Reset the flag so init() knows it has to start over next time
    _vision_ready = False
    
    print("[VISION] Shutdown complete. Cameras released.")