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

# Constants for single-camera distance estimation
BOTTLE_REAL_HEIGHT_CM = 21.5
FOCAL_LENGTH = 410.36  # Replace with your calibrated focal length

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
    R, T = data['R'], data['T']
    
    img_size = (640, 480) 
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, alpha=-1)
    
    map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    
    # We return the maps. We no longer need the Q matrix for this method.
    return map_l, map_r

def calculate_distance_by_height(pixel_height):
    """Calculate distance using the pinhole camera model formula."""
    if pixel_height <= 0: return 0
    return (BOTTLE_REAL_HEIGHT_CM * FOCAL_LENGTH) / pixel_height

# ==========================================
# MAIN LOOP
# ==========================================

def main():
    maps_l, maps_r = load_calibration(CALIBRATION_FILE_PATH)
    cap_l = cv2.VideoCapture(CAMERA_LEFT_INDEX)
    cap_r = cv2.VideoCapture(CAMERA_RIGHT_INDEX)

    print("Starting continuous detection on LEFT camera. Press 'q' to stop.")

    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            print("Error reading from cameras.")
            break

        # Rectify images to remove lens distortion
        rect_l = cv2.remap(frame_l, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

        # Detect bottles ONLY on the left camera to save CPU/GPU resources
        res_l = model.predict(rect_l, conf=0.5, classes=[BOTTLE_CLASS_ID], verbose=False)

        current_distance = "N/A"

        # Check if a bottle was detected in the left frame
        if res_l and len(res_l[0].boxes) > 0:
            # Get data of the first detected bottle
            box_l = res_l[0].boxes[0]
            x_l, y_l, w_l, h_l = box_l.xywh[0]
            
            # Use the height in pixels (h_l) to calculate distance
            distance = calculate_distance_by_height(float(h_l))
            current_distance = f"{distance:.1f} cm"
            
            # Draw bounding box and target center on left image
            cv2.rectangle(rect_l, (int(x_l-w_l/2), int(y_l-h_l/2)), 
                          (int(x_l+w_l/2), int(y_l+h_l/2)), (0, 255, 0), 2)

        # Visualization: Show both cameras side-by-side
        display = np.hstack((rect_l, rect_r))
        cv2.putText(display, f"Distance (Left Cam): {current_distance}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow("Distance Measurement (Height Based)", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()