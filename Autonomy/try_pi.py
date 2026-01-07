# -*- coding: utf-8 -*-
import time
import cv2
from ultralytics import YOLO

# --- CONFIGURATION ---
LEFT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"
RIGHT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
MODEL_NAME = "yolov8n.pt"

# Capture settings (Pi-friendly)
W, H = 640, 480
CAM_FPS = 15  # matches what you negotiated
CONF = 0.5
IMGSZ = 320   # smaller = faster on Pi

# How long to run (seconds). Set to 0 or None to run forever.
RUN_SECONDS = 0

def open_cam(path: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"Opening: {path}")
    print("  isOpened:", cap.isOpened())
    if not cap.isOpened():
        return cap

    # Prefer MJPG for USB cams (usually better FPS on Pi)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Warm up
    for _ in range(5):
        cap.read()

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"  negotiated: {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.1f}fps")
    return cap

def count_dets(result) -> int:
    """Count detections in an Ultralytics result object."""
    try:
        b = result.boxes
        if b is None:
            return 0
        # boxes.cls is a tensor of class ids (N,)
        return int(len(b.cls))
    except Exception:
        return 0

def main():
    print("Initializing headless YOLOv8 stereo test...")

    cap_l = open_cam(LEFT)
    cap_r = open_cam(RIGHT)

    if not cap_l.isOpened() or not cap_r.isOpened():
        print("Error: Could not open one or both cameras.")
        return

    model = YOLO(MODEL_NAME)
    print(f"Model {MODEL_NAME} loaded.")
    print("Running headless (no display). Ctrl+C to stop.")

    start = time.time()
    prev = start
    frames = 0

    try:
        while True:
            if RUN_SECONDS and (time.time() - start) >= RUN_SECONDS:
                print("Reached RUN_SECONDS limit. Stopping.")
                break

            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()

            if not ret_l or frame_l is None:
                print("Left camera: failed to grab frame.")
                break
            if not ret_r or frame_r is None:
                print("Right camera: failed to grab frame.")
                break

            # Inference (headless) — no plotting, no imshow
            results_l = model.predict(frame_l, conf=CONF, imgsz=IMGSZ, verbose=False)
            results_r = model.predict(frame_r, conf=CONF, imgsz=IMGSZ, verbose=False)

            dets_l = count_dets(results_l[0])
            dets_r = count_dets(results_r[0])

            frames += 1
            now = time.time()
            if now - prev >= 1.0:
                fps = frames / (now - start)
                print(f"[{now - start:6.1f}s] avg_fps={fps:5.2f}  dets(L,R)=({dets_l},{dets_r})")
                prev = now

    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C).")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        cap_l.release()
        cap_r.release()
        print("Cameras released.")

if __name__ == "__main__":
    main()
