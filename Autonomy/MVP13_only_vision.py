import cv2
import pickle
from ultralytics import YOLO
import time

# --- Vision Configurations ---
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

W = 640
H = 480
BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50

# --- Global Variables ---
_vision_ready = False
_model = _cap_l = _cap_r = _maps_l = _maps_r = _Q = None

def init_vision():
    global _vision_ready, _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    if _vision_ready: 
        return
        
    # Initialize YOLO model
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')
    
    # Initialize stereo cameras
    _cap_l = cv2.VideoCapture(LEFT_CAM, cv2.CAP_V4L2)
    _cap_r = cv2.VideoCapture(RIGHT_CAM, cv2.CAP_V4L2)
    
    # Load stereo calibration data
    with open(CALIBRATION_FILE_PATH, 'rb') as f: 
        data = pickle.load(f)
        
    _Q = data['Q']
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        data['cameraMatrix1'], data['distCoeffs1'], 
        data['cameraMatrix2'], data['distCoeffs2'], 
        (W, H), data['R'], data['T'], alpha=-1
    )
    
    # Initialize rectification maps
    _maps_l = cv2.initUndistortRectifyMap(data['cameraMatrix1'], data['distCoeffs1'], R1, P1, (W, H), cv2.CV_32FC1)
    _maps_r = cv2.initUndistortRectifyMap(data['cameraMatrix2'], data['distCoeffs2'], R2, P2, (W, H), cv2.CV_32FC1)
    
    _vision_ready = True

def detect_bottle_once():
    global _model, _cap_l, _cap_r, _maps_l, _maps_r
    
    # Grab frames to clear buffer and get the most recent ones
    for _ in range(5): 
        _cap_l.grab()
        _cap_r.grab()
        
    _, fl = _cap_l.retrieve()
    _, fr = _cap_r.retrieve()
    
    # Rectify left frame for YOLO prediction
    f_rect = cv2.remap(fl, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
    
    # Predict using YOLO
    res = _model.predict(f_rect, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)
    
    X = Y = Z = None
    
    if res and len(res[0].boxes) > 0:
        # Note: These are placeholder values.
        # Actual stereo triangulation logic should replace this to get real depth (X, Y, Z).
        X, Y, Z = 0, 0, 100 
        
    return {"found": X is not None, "X": X, "Y": Y, "Z": Z}

def track_bottle_continuously():
    # Initialize the cameras, calibration data, and YOLO model
    init_vision()
    print("Starting continuous bottle tracking. Press Ctrl+C to stop.")
    
    try:
        while True:
            # Perform detection
            result = detect_bottle_once()
            
            # Print the results continuously
            if result["found"]:
                print(f"Bottle Detected! Distance -> X: {result['X']}, Y: {result['Y']}, Z: {result['Z']}")
            else:
                print("Looking for bottle...")
                
            # Small delay to prevent console flooding and CPU overload
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        print("\nStopping tracking...")
    finally:
        # Release cameras properly on exit
        if _cap_l: _cap_l.release()
        if _cap_r: _cap_r.release()

if __name__ == "__main__":
    track_bottle_continuously()