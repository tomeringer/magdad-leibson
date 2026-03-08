# -*- coding: utf-8 -*-
import cv2
import numpy as np
import pickle
from ultralytics import YOLO

# ==========================================
# SETTINGS
# ==========================================
CALIBRATION_FILE_PATH = "C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_calibration.pkl"
CAMERA_LEFT_INDEX = 1 
CAMERA_RIGHT_INDEX = 2 
BOTTLE_CLASS_ID = 39 

print("Loading YOLO model...")
model = YOLO('yolov8n.pt')

# ==========================================
# FUNCTIONS
# ==========================================

def load_calibration(file_path):
    """Load stereo parameters and create rectification maps."""
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    K1, D1 = data['cameraMatrix1'], data['distCoeffs1']
    K2, D2 = data['cameraMatrix2'], data['distCoeffs2']
    R, T, Q = data['R'], data['T'], data['Q']
    
    img_size = (640, 480) 
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, alpha=-1)
    
    map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    return map_l, map_r, Q

def calculate_depth(disparity, x, y, Q_matrix):
    """Calculate Z distance using disparity and Q matrix."""
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    homogeneous_point = np.dot(Q_matrix, point)
    w = homogeneous_point[3]
    if w == 0: return 0
    return abs(homogeneous_point[2] / w)

# ==========================================
# MAIN LOOP
# ==========================================

def main():
    maps_l, maps_r, Q = load_calibration(CALIBRATION_FILE_PATH)
    cap_l = cv2.VideoCapture(CAMERA_LEFT_INDEX)
    cap_r = cv2.VideoCapture(CAMERA_RIGHT_INDEX)

    print("Starting continuous detection. Press 'q' to stop.")

    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            break

        # Rectify images
        rect_l = cv2.remap(frame_l, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

        # Detect bottles
        res_l = model.predict(rect_l, conf=0.5, classes=[BOTTLE_CLASS_ID], verbose=False)
        res_r = model.predict(rect_r, conf=0.5, classes=[BOTTLE_CLASS_ID], verbose=False)

        current_distance = "N/A"

        if res_l and len(res_l[0].boxes) > 0 and res_r and len(res_r[0].boxes) > 0:
            # Get center of first detected bottle in left camera
            box_l = res_l[0].boxes[0]
            x_l, y_l, w_l, h_l = box_l.xywh[0]
            
            # Find matching box in right camera (simple epipolar match)
            for box_r in res_r[0].boxes:
                x_r, y_r = box_r.xywh[0][0], box_r.xywh[0][1]
                if abs(y_l - y_r) < 15: # Horizontal alignment check
                    disparity = abs(x_l - x_r)
                    if disparity > 0:
                        z = calculate_depth(disparity, int(x_l), int(y_l), Q)
                        current_distance = f"{z:.1f} cm"
                        
                        # Draw detection on left image
                        cv2.rectangle(rect_l, (int(x_l-w_l/2), int(y_l-h_l/2)), 
                                      (int(x_l+w_l/2), int(y_l+h_l/2)), (0, 255, 0), 2)

        # Visualization
        display = np.hstack((rect_l, rect_r))
        cv2.putText(display, f"Distance: {current_distance}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow("Stereo Distance Measurement", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()