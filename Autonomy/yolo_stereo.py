import math
import pickle
import time
from datetime import datetime
import threading

import cv2
import numpy as np
from ultralytics import YOLO

import lib_logger


def open_cam(path: str, name: str, w: int, h: int, fps: int):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"{name}: opening {path}  isOpened={cap.isOpened()}")
    if not cap.isOpened():
        return cap

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Try to keep buffers tiny (driver may still buffer more, but this helps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Warm up (and flush)
    for _ in range(20):
        cap.grab()

    print(f"{name}: negotiated "
          f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.1f}fps "
          f"fourcc={int(cap.get(cv2.CAP_PROP_FOURCC))}")
    return cap


class LatestStereo:
    """
    Continuously grabs from both cameras and stores ONLY the latest synchronized pair.
    This is the #1 way to kill multi-second latency on Pi + UVC.
    """
    def __init__(self, cap_l: cv2.VideoCapture, cap_r: cv2.VideoCapture):
        self.cap_l = cap_l
        self.cap_r = cap_r
        self.lock = threading.Lock()
        self.latest = None  # (t_capture, frame_l, frame_r)
        self.running = False
        self.th = None

    def start(self):
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()
        return self

    def stop(self):
        self.running = False
        if self.th:
            self.th.join(timeout=1.0)

    def _loop(self):
        while self.running:
            # Grab both first (better sync, also avoids decode until retrieve)
            ok_l = self.cap_l.grab()
            ok_r = self.cap_r.grab()
            if not ok_l or not ok_r:
                continue

            ok_l, frame_l = self.cap_l.retrieve()
            ok_r, frame_r = self.cap_r.retrieve()
            if not ok_l or not ok_r:
                continue

            tcap = time.perf_counter()
            with self.lock:
                self.latest = (tcap, frame_l, frame_r)

    def read_latest(self):
        with self.lock:
            return self.latest


def load_and_prepare_calibration(file_path, img_size):
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
        print(f"Error loading calibration file: {e}")
        return None, None, None, None

    print(f"Using image size: {img_size}")

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=img_size, R=R, T=T,
        alpha=-1
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)

    return (map1_l, map2_l), (map1_r, map2_r), Q, img_size


def calculate_3d_coords(disparity, x, y, Q_matrix):
    # NOTE: disparity sign matters; do NOT abs() if you use Q properly.
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    hp = Q_matrix @ point
    w = hp[3]
    if w == 0:
        return (0.0, 0.0, 0.0)
    return (float(hp[0] / w), float(hp[1] / w), float(hp[2] / w))


if __name__ == '__main__':
    ts = datetime.now().strftime("%Y%m%d_%H%M")
    metrics = lib_logger.CSVMetricLogger(
        f"cam_logs/log{ts}.csv",
        fieldnames=["t_unix", "frame_age_s", "bottle_x", "bottle_y", "bottle_z", "t_proc"]
    )

    CALIBRATION_FILE_PATH = "stereo_calibration.pkl"

    LEFT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
    RIGHT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

    W, H = 640, 480
    CAM_FPS = 15
    IMG_SIZE = (W, H)

    model = YOLO('yolov8n_ncnn_model', task='detect')
    BOTTLE_CLASS_ID = 39

    cap_l = open_cam(LEFT, "LEFT", W, H, CAM_FPS)
    cap_r = open_cam(RIGHT, "RIGHT", W, H, CAM_FPS)

    maps_l, maps_r, Q_matrix, _ = load_and_prepare_calibration(CALIBRATION_FILE_PATH, IMG_SIZE)
    if maps_l is None:
        raise SystemExit(1)

    stereo = LatestStereo(cap_l, cap_r).start()

    print("Starting YOLOv8 Stereo Detection (low latency).")

    try:
        while True:
            t0 = time.perf_counter()

            pack = stereo.read_latest()
            if pack is None:
                continue

            tcap, frame_l_orig, frame_r_orig = pack
            frame_age_s = time.perf_counter() - tcap  # this is your true latency indicator

            # Rectify
            frame_l_rect = cv2.remap(frame_l_orig, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
            frame_r_rect = cv2.remap(frame_r_orig, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

            # Batched YOLO (one call instead of two)
            results = model.predict([frame_l_rect, frame_r_rect],
                                    conf=0.50,
                                    classes=[BOTTLE_CLASS_ID],
                                    verbose=False)
            results_l, results_r = results[0], results[1]

            X = Y = Z = None

            if results_l.boxes is not None and len(results_l.boxes) > 0:
                box_l = results_l.boxes[0]
                x_l = int(box_l.xywh[0][0].item())
                y_l = int(box_l.xywh[0][1].item())

                # Find a matching box in right frame by y proximity
                best = None
                for box_r in results_r.boxes:
                    x_r = int(box_r.xywh[0][0].item())
                    y_r = int(box_r.xywh[0][1].item())
                    if abs(y_l - y_r) < 10:
                        best = (x_r, y_r)
                        break

                if best is not None:
                    x_r, _ = best

                    # IMPORTANT: use signed disparity for Q reprojection
                    disparity = float(x_l - x_r)

                    if abs(disparity) > 0.5:
                        X, Y, Z = calculate_3d_coords(disparity, float(x_l), float(y_l), Q_matrix)
                        print(f"X:{X:.1f}  Y:{Y:.1f}  Z:{Z:.1f}   (age {frame_age_s:.3f}s)")

            t_total = time.perf_counter() - t0
            metrics.log(frame_age_s=frame_age_s, bottle_x=X, bottle_y=Y, bottle_z=Z, t_proc=t_total)

    finally:
        stereo.stop()
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print("Program terminated. Resources released.")
