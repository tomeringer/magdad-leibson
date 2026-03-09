import cv2
import numpy as np
import pickle
import time
import math
from ultralytics import YOLO

# ============================================================
# PC CONFIGURATION
# ============================================================
# Use integer indices for PC webcams (e.g., 0, 1, 2)
LEFT_CAM = 1
RIGHT_CAM = 2

CALIBRATION_FILE_PATH = "C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_calibration.pkl"

W = 640
H = 480
CAM_FPS = 15
DEFAULT_IMG_SIZE = (W, H)

# COCO bottle class
BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50

# Epipolar match tolerance in rectified images
MATCH_Y_TOL = 10

# Global cached vision state
_vision_ready = False
_model = None
_cap_l = None
_cap_r = None
_maps_l = None
_maps_r = None
_Q = None

# ============================================================
# VISION FUNCTIONS
# ============================================================
def open_cam(camera_index: int, name: str):
    # Using CAP_DSHOW for faster startup on Windows. Change to CAP_ANY if on Linux/Mac.
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW) 
    print(f"[CAM] {name}: opening index {camera_index} isOpened={cap.isOpened()}", flush=True)
    if not cap.isOpened():
        return cap

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    # Warm up cameras
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

    print("[VISION] Initializing YOLO + stereo on PC...", flush=True)

    # Load YOLO model
    _model = YOLO('yolov8n.pt', task='detect')

    _cap_l = open_cam(LEFT_CAM, "LEFT")
    _cap_r = open_cam(RIGHT_CAM, "RIGHT")
    if not _cap_l.isOpened() or not _cap_r.isOpened():
        raise RuntimeError("One or both cameras failed to open. Check camera indices.")

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

    ret_l, frame_l_orig, ret_r, frame_r_orig = read_latest_stereo(
        _cap_l, _cap_r, flush_n=5
    )

    if not ret_l or not ret_r:
        return {"found": False, "err": "Failed to read frames", "t_proc": time.perf_counter() - t0, "frame_l": None, "frame_r": None}

    # Rectify frames
    frame_l = cv2.remap(frame_l_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    frame_r = cv2.remap(frame_r_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

    # YOLO predictions
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
        
        # Get bounding box coordinates for left frame
        x1_l, y1_l, x2_l, y2_l = map(int, box_l.xyxy[0])

        closest_box_r = None
        if results_r and len(results_r[0].boxes) > 0:
            for box_r in results_r[0].boxes:
                y_r = int(box_r.xywh[0][1].item())
                if abs(cy - y_r) < MATCH_Y_TOL:
                    closest_box_r = box_r
                    break

        if closest_box_r is not None:
            x_r = int(closest_box_r.xywh[0][0].item())
            disparity = abs(cx - x_r)
            
            # Get bounding box coordinates for right frame
            x1_r, y1_r, x2_r, y2_r = map(int, closest_box_r.xyxy[0])
            
            # Draw rectangle on right frame
            cv2.rectangle(frame_r, (x1_r, y1_r), (x2_r, y2_r), (255, 0, 0), 2)
            cv2.putText(frame_r, "Matched", (x1_r, y1_r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            if disparity and disparity > 0:
                R = 0
                X, Y, R = calculate_3d_coords(disparity, cx, cy, _Q)
                Z = R

                alpha = math.atan2(X, Z)
                r = math.hypot(X, Z)
                alpha = alpha + math.radians(3)
                X = math.sin(alpha) * r
                Z = math.cos(alpha) * r
                
                # Draw rectangle and 3D data on left frame
                cv2.rectangle(frame_l, (x1_l, y1_l), (x2_l, y2_l), (0, 255, 0), 2)
                text = f"X:{X:.1f} Y:{Y:.1f} Z:{Z:.1f}"
                cv2.putText(frame_l, text, (x1_l, y1_l - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Draw rectangle on left frame even if no match is found (no depth data)
            cv2.rectangle(frame_l, (x1_l, y1_l), (x2_l, y2_l), (0, 165, 255), 2)
            cv2.putText(frame_l, "No Match", (x1_l, y1_l - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    t_proc = time.perf_counter() - t0
    
    # Return frames along with data
    return {
        "found": X is not None,
        "X": X, "Y": Y, "Z": Z,
        "cx": cx, "cy": cy,
        "disparity": disparity,
        "conf": conf,
        "t_proc": t_proc,
        "frame_l": frame_l,
        "frame_r": frame_r
    }


def shutdown_vision():
    global _cap_l, _cap_r
    try:
        if _cap_l is not None:
            _cap_l.release()
        if _cap_r is not None:
            _cap_r.release()
        cv2.destroyAllWindows()
    except Exception:
        pass


# ============================================================
# MAIN PC LOOP
# ============================================================
def track_bottle_on_pc():
    try:
        init_vision()
        print("\n[INFO] Starting continuous tracking... Press 'q' on the video window to quit.\n")
        
        while True:
            result = detect_bottle_once()
            
            frame_l = result.get("frame_l")
            frame_r = result.get("frame_r")
            
            # Display frames if they were successfully read
            if frame_l is not None and frame_r is not None:
                # Add labels to the frames
                cv2.putText(frame_l, "LEFT CAM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame_r, "RIGHT CAM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Combine frames side-by-side
                combined_frame = np.hstack((frame_l, frame_r))
                cv2.imshow("Stereo Vision PC - Press 'q' to exit", combined_frame)
            
            if result["found"]:
                print(f"[DETECTED] Bottle -> X: {result['X']:.2f}, Y: {result['Y']:.2f}, Z: {result['Z']:.2f} | Disp: {result['disparity']} | Time: {result['t_proc']:.3f}s")
            
            # Wait for 1 ms and check if 'q' is pressed to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[INFO] 'q' pressed. Exiting loop.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Stopping tracking process...")
    finally:
        shutdown_vision()
        print("[OFF] Cameras released. System Stopped.")


if __name__ == "__main__":
    track_bottle_on_pc()