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

    # Use MJPG to reduce USB bandwidth (usually lowers latency too)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Try to keep buffers tiny (driver may still buffer more, but this helps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Warm up / flush
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
    This drops old frames and kills multi-second latency.
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


def load_and_prepare_calibration(file_path: str, img_size: tuple[int, int]):
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

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=img_size, R=R, T=T,
        alpha=-1
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)

    return (map1_l, map2_l), (map1_r, map2_r), Q, img_size


def calculate_3d_coords(disparity: float, x: float, y: float, Q_matrix: np.ndarray):
    point = np.array([x, y, disparity, 1.0], dtype=np.float32)
    hp = Q_matrix @ point
    w = float(hp[3])
    if w == 0.0:
        return (0.0, 0.0, 0.0)
    return (float(hp[0] / w), float(hp[1] / w), float(hp[2] / w))


def pick_best_box(results, prefer_center=True):
    if results is None or results.boxes is None or len(results.boxes) == 0:
        return None

    boxes = results.boxes
    if hasattr(boxes, "conf") and boxes.conf is not None:
        order = np.argsort(-boxes.conf.cpu().numpy())
        boxes = boxes[order]

    return boxes[0]


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
    BOTTLE_CLASS_ID = 39

    model = YOLO('yolov8n_ncnn_model', task='detect')

    cap_l = open_cam(LEFT, "LEFT", W, H, CAM_FPS)
    cap_r = open_cam(RIGHT, "RIGHT", W, H, CAM_FPS)

    maps_l, maps_r, Q_matrix, img_size = load_and_prepare_calibration(CALIBRATION_FILE_PATH, IMG_SIZE)
    if maps_l is None:
        raise SystemExit(1)

    stereo = LatestStereo(cap_l, cap_r).start()
    print("Starting YOLOv8 Stereo Detection (low latency, non-batched NCNN).")

    try:
        while True:
            t0 = time.perf_counter()
            pack = stereo.read_latest()
            if pack is None:
                continue

            tcap, frame_l_orig, frame_r_orig = pack
            frame_age_s = time.perf_counter() - tcap

            # Rectify
            frame_l_rect = cv2.remap(frame_l_orig, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
            frame_r_rect = cv2.remap(frame_r_orig, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

            # YOLO left image
            res_l = model.predict(frame_l_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)[0]
            box_l = pick_best_box(res_l)

            if box_l is not None:
                x_l = float(box_l.xywh[0][0].item())
                y_l = float(box_l.xywh[0][1].item())
                y = int(round(y_l))

                # Epipolar band
                band = 20
                y0 = max(0, y - band)
                y1 = min(H, y + band)

                strip_l = frame_l_rect[y0:y1, :]
                strip_r = frame_r_rect[y0:y1, :]

                gL = cv2.cvtColor(strip_l, cv2.COLOR_BGR2GRAY)
                gR = cv2.cvtColor(strip_r, cv2.COLOR_BGR2GRAY)

                tpl_w = 40
                x0 = max(0, int(x_l - tpl_w // 2))
                x1 = min(W, x0 + tpl_w)
                template = gL[:, x0:x1]

                res = cv2.matchTemplate(gR, template, cv2.TM_SQDIFF)
                _, _, min_loc, _ = cv2.minMaxLoc(res)

                x_r = float(min_loc[0] + template.shape[1] / 2)
                disparity = x_l - x_r

                if abs(disparity) > 0.5:
                    X, Y, Z = calculate_3d_coords(disparity, x_l, y_l, Q_matrix)
                    print(f"X:{X:.1f}  Y:{Y:.1f}  Z:{Z:.1f}   (age {frame_age_s:.3f}s)")
                else:
                    X = Y = Z = None
            else:
                X = Y = Z = None
                print(f"No bottle detected (frame age {frame_age_s:.3f}s)")

            t_total = time.perf_counter() - t0
            metrics.log(frame_age_s=frame_age_s, bottle_x=X, bottle_y=Y, bottle_z=Z, t_proc=t_total)

    finally:
        stereo.stop()
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print("Program terminated. Resources released.")
