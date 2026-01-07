# -*- coding: utf-8 -*-
import cv2
import numpy as np
import pickle
import time
import RPi.GPIO as GPIO
from gpiozero import OutputDevice
from ultralytics import YOLO

# ==========================================
# HARDWARE CONFIGURATION (BCM Numbering)
# ==========================================
GPIO.setmode(GPIO.BCM)

# DC Motors (IBT-2 Driver)
M1_RPWM = OutputDevice(18, active_high=True, initial_value=False)
M1_LPWM = OutputDevice(23, active_high=True, initial_value=False)
M2_RPWM = OutputDevice(24, active_high=True, initial_value=False)
M2_LPWM = OutputDevice(25, active_high=True, initial_value=False)

# Stepper Motor
STEPPER_STEP = OutputDevice(17, active_high=True, initial_value=False)
STEPPER_DIR = OutputDevice(27, active_high=True, initial_value=False)

def motor_stop():
    """Stop all DC motor movement."""
    M1_RPWM.off(); M1_LPWM.off()
    M2_RPWM.off(); M2_LPWM.off()
    print("Motors: STOP")

def move_forward():
    """Drive both DC motors forward."""
    M1_RPWM.on(); M1_LPWM.off()
    M2_RPWM.on(); M2_LPWM.off()
    print("Motors: FORWARD")

# ==========================================
# VISION & AUTONOMY SETTINGS
# ==========================================
CALIBRATION_FILE_PATH = "stereo_calibration.pkl"
CAMERA_LEFT_INDEX = 1
CAMERA_RIGHT_INDEX = 0
BOTTLE_CLASS_ID = 39 

# Autonomous Logic constants
ROBOT_SPEED_CM_S = 15.0  # Measure and update this (cm per second)
STOP_DISTANCE_CM = 10.0  # Target distance to stop from the bottle

# Load YOLOv8 model
print("Loading YOLO model...")
model = YOLO('yolov8n.pt')

# ==========================================
# STEREO VISION FUNCTIONS
# ==========================================

def load_calibration(file_path):
    """Load stereo parameters and create rectification maps."""
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    K1, D1 = data['cameraMatrix1'], data['distCoeffs1']
    K2, D2 = data['cameraMatrix2'], data['distCoeffs2']
    R, T, Q = data['R'], data['T'], data['Q']
    
    img_size = (640, 480) # Default calibration size
    
    # Compute rectification transforms
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, alpha=-1)
    
    # Initialize undistortion and rectification maps
    map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    return map_l, map_r, Q

def calculate_3d_coords(disparity, x, y, Q_matrix):
    """Reproject image points to 3D space using the Q matrix."""
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    homogeneous_point = np.dot(Q_matrix, point)
    w = homogeneous_point[3]
    if w == 0: return (0, 0, 0)
    return (homogeneous_point[0]/w, homogeneous_point[1]/w, homogeneous_point[2]/w)

def get_bottle_distance(cap_l, cap_r, maps_l, maps_r, Q_matrix):
    """Detect bottle in both frames and calculate Z depth."""
    ret_l, frame_l = cap_l.read()
    ret_r, frame_r = cap_r.read()
    if not ret_l or not ret_r: return None

    # Apply rectification maps
    rect_l = cv2.remap(frame_l, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
    rect_r = cv2.remap(frame_r, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

    # YOLO Inference
    res_l = model.predict(rect_l, conf=0.5, classes=[BOTTLE_CLASS_ID], verbose=False)
    res_r = model.predict(rect_r, conf=0.5, classes=[BOTTLE_CLASS_ID], verbose=False)

    if res_l and len(res_l[0].boxes) > 0 and res_r and len(res_r[0].boxes) > 0:
        box_l = res_l[0].boxes[0]
        x_l, y_l = int(box_l.xywh[0][0]), int(box_l.xywh[0][1])

        # Match with right frame detection (Epipolar constraint)
        for box_r in res_r[0].boxes:
            x_r, y_r = int(box_r.xywh[0][0]), int(box_r.xywh[0][1])
            if abs(y_l - y_r) < 15: 
                disparity = abs(x_l - x_r)
                if disparity > 0:
                    _, _, Z = calculate_3d_coords(disparity, x_l, y_l, Q_matrix)
                    return Z
    return None

# ==========================================
# MAIN AUTONOMOUS EXECUTION
# ==========================================

def main():
    cap_l = None
    cap_r = None
    try:
        # Preparation
        maps_l, maps_r, Q = load_calibration(CALIBRATION_FILE_PATH)
        cap_l = cv2.VideoCapture(CAMERA_LEFT_INDEX)
        cap_r = cv2.VideoCapture(CAMERA_RIGHT_INDEX)
        
        print("System initialized. Searching for target bottle...")
        
        while True:
            z_distance = get_bottle_distance(cap_l, cap_r, maps_l, maps_r, Q)
            
            if z_distance is not None:
                z_distance = abs(z_distance) # Ensure positive value
                print(f"Target found at {z_distance:.1f} cm")
                
                # Calculate required movement
                dist_to_drive = z_distance - STOP_DISTANCE_CM
                
                if dist_to_drive > 0:
                    drive_time = dist_to_drive / ROBOT_SPEED_CM_S
                    print(f"Action Plan: Drive forward for {drive_time:.2f} seconds")
                    
                    # Execute hardware movement
                    move_forward()
                    time.sleep(drive_time)
                    motor_stop()
                    
                    # Log activity to your data file
                    with open("finally", "a") as f:
                        f.write(f"Mission Success: Initially {z_distance:.2f}cm away. Drove {dist_to_drive:.2f}cm.\n")
                    
                    print("Reached target location. Mission completed.")
                    break
                else:
                    print("Already at or inside the target stop distance.")
                    break
            
            time.sleep(0.1) # Loop delay to prevent CPU overload

    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    finally:
        motor_stop()
        GPIO.cleanup()
        if cap_l: cap_l.release()
        if cap_r: cap_r.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()