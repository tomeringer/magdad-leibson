# -*- coding: utf-8 -*-
import pickle
import cv2

# Path to your existing calibration file
CALIBRATION_FILE_PATH = "C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_calibration.pkl"

def extract_focal_length():
    # Open and load the pickle file
    with open(CALIBRATION_FILE_PATH, 'rb') as f:
        data = pickle.load(f)
    
    # Extract matrices from the dictionary
    K1, D1 = data['cameraMatrix1'], data['distCoeffs1']
    K2, D2 = data['cameraMatrix2'], data['distCoeffs2']
    R, T = data['R'], data['T']
    
    # Resolution used during calibration
    img_size = (640, 480) 
    
    # Calculate rectification matrices
    _, _, P1, _, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, alpha=-1)
    
    # The focal length for the Y-axis (height) is located at index [1, 1] of the Projection Matrix P1
    focal_length_y = P1[1, 1]
    
    print("-" * 40)
    print(f"SUCCESS! Your FOCAL_LENGTH is: {focal_length_y:.2f}")
    print("-" * 40)

if __name__ == "__main__":
    extract_focal_length()