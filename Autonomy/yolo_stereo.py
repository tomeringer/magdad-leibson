# =========================
# FILE: vision_pub.py
# (Your YOLO+stereo code + ZeroMQ PUB)
# =========================

import math
import pickle
import time
from datetime import datetime
import json

import cv2
import numpy as np
from ultralytics import YOLO

import lib_logger

import zmq


# --- FUNCTION TO LOAD AND PREPARE CALIBRATION DATA ---
def open_cam(path: str, name: str):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"{name}: opening {path}  isOpened={cap.isOpened()}")
    if not cap.isOpened():
        return cap

    # Force MJPG (compressed) to reduce USB bandwidth
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

    # Keep a small buffer; 1 can be too aggressive on some drivers
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    # Warm up
    for _ in range(10):
        cap.read()

    print(f"{name}: negotiated "
          f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.1f}fps "
          f"fourcc={int(cap.get(cv2.CAP_PROP_FOURCC))}")
    return cap


def load_and_prepare_calibration(file_path):
    """
    Loads stereo parameters (K, D, R, T, Q) from the .pkl file
    and computes the necessary rectification maps (R1, P1, etc.).
    """
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)

        # *** Adjusted to match the keys in your stereo_calibration.pkl file ***
        K1 = data['cameraMatrix1']
        D1 = data['distCoeffs1']
        K2 = data['cameraMatrix2']
        D2 = data['distCoeffs2']
        R = data['R']
        T = data['T']
        Q = data['Q']  # Q matrix is already computed

    except Exception as e:
        print(f"Error loading calibration file: {e}")
        print("Ensure the file exists and the internal keys (cameraMatrix1, R, T, Q etc.) are correct.")
        return None, None, None, None

    img_size = DEFAULT_IMG_SIZE  # (width, height)
    print(f"Using image size: {img_size}")

    # Recompute rectification using stereoRectify
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=img_size, R=R, T=T,
        alpha=-1
    )

    # Create the Rectification Maps
    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)

    print("Stereo calibration loaded and rectification maps created successfully.")
    return (map1_l, map2_l), (map1_r, map2_r), Q, img_size


# --- FUNCTION TO CALCULATE 3D DEPTH ---
def calculate_3d_coords(disparity, x, y, Q_matrix):
    """
    Computes 3D coordinates (X, Y, Z) from a pixel (x, y) and its disparity value.
    """
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    homogeneous_point = np.dot(Q_matrix, point)

    w = homogeneous_point[3]
    if w == 0:
        return (0, 0, 0)

    X = homogeneous_point[0] / w
    Y = homogeneous_point[1] / w
    Z = homogeneous_point[2] / w

    return (X, Y, Z)


if __name__ == '__main__':
    # --- CONFIGURATION ---
    ts = datetime.now().strftime("%Y%m%d_%H%M")
    metrics = lib_logger.CSVMetricLogger(
        f"cam_logs/log{ts}.csv",
        fieldnames=["t_unix", "bottle_x", "bottle_y", "bottle_z", "t_proc"]
    )

    # Path to your generated stereo calibration file
    CALIBRATION_FILE_PATH = r"stereo_calibration.pkl"

    # Indices of your USB cameras (must match the cameras used for calibration)
    LEFT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
    RIGHT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

    # Image size
    W = 640
    H = 480
    CAM_FPS = 15
    DEFAULT_IMG_SIZE = (W, H)

    # Load the YOLOv8 model
    # model = YOLO('yolov8n.pt')
    model = YOLO('yolov8n_ncnn_model', task='detect')

    # COCO dataset class ID for 'bottle'
    BOTTLE_CLASS_ID = 39

    cap_l = open_cam(LEFT, "LEFT")
    cap_r = open_cam(RIGHT, "RIGHT")

    # Load and prepare calibration data
    maps_l, maps_r, Q_matrix, img_size = load_and_prepare_calibration(CALIBRATION_FILE_PATH)
    if maps_l is None:
        exit()

    # ================= ZERO MQ PUBLISHER =================
    ZMQ_ADDR = "tcp://*:5555"
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 1)  # no backlog
    pub.bind(ZMQ_ADDR)
    time.sleep(0.2)
    print("[ZMQ] Publisher bound on", ZMQ_ADDR)

    print("Starting YOLOv8 Stereo Detection (Publisher).")

    while True:
        t0 = time.perf_counter()

        ret_l, frame_l_orig = cap_l.read()
        ret_r, frame_r_orig = cap_r.read()

        if not ret_l or not ret_r:
            print("Error: Failed to read frames. Check camera connection.")
            break

        # --- 1. Rectification ---
        frame_l_rect = cv2.remap(frame_l_orig, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
        frame_r_rect = cv2.remap(frame_r_orig, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

        # --- 2. YOLO Detection ---
        results_l = model.predict(frame_l_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)
        results_r = model.predict(frame_r_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)

        # --- 3. 3D Depth Calculation ---
        X, Y, Z = None, None, None
        cx, cy = None, None

        if results_l and len(results_l[0].boxes) > 0:
            box_l = results_l[0].boxes[0]
            cx, cy = int(box_l.xywh[0][0].item()), int(box_l.xywh[0][1].item())

            closest_box_r = None
            if results_r and len(results_r[0].boxes) > 0:
                for box_r in results_r[0].boxes:
                    x_r, y_r = int(box_r.xywh[0][0].item()), int(box_r.xywh[0][1].item())
                    if abs(cy - y_r) < 10:
                        closest_box_r = box_r
                        break

            if closest_box_r is not None:
                x_r = int(closest_box_r.xywh[0][0].item())
                disparity = abs(cx - x_r)

                if disparity > 0:
                    # NOTE: your original code computed Z in a nonstandard way.
                    # We'll keep it as-is to avoid changing behavior, but you likely want
                    # to just use the Z returned by calculate_3d_coords.
                    X, Y, R = calculate_3d_coords(disparity, cx, cy, Q_matrix)
                    try:
                        Z = math.sqrt(max(0.0, R**2 - X**2 - Y**2))
                    except Exception:
                        Z = None

                    print(f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: {Z:.1f}cm" if Z is not None else
                          f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: None")

        # --- Publish ---
        msg = {
            "t": time.time(),
            "found": X is not None,
            "X": X,
            "Y": Y,
            "Z": Z,
            "cx": cx,
            "cy": cy,
            "w": W,
            "h": H,
        }
        pub.send_multipart([b"bottle", json.dumps(msg).encode("utf-8")])

        t_total = time.perf_counter() - t0
        metrics.log(t_unix=time.time(), bottle_x=X, bottle_y=Y, bottle_z=Z, t_proc=t_total)

    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()
    print("Program terminated. Resources released.")
