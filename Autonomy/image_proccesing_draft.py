import cv2
import numpy as np
import pickle
from ultralytics import YOLO

# --- CONFIGURATION ---
# Path to your generated stereo calibration file
CALIBRATION_FILE_PATH = r"C:\Users\TLP-001\Desktop\magdad-leibson\Autonomy\stereo_calibration.pkl"

# Indices of your USB cameras (must match the cameras used for calibration)
CAMERA_LEFT_INDEX = 1
CAMERA_RIGHT_INDEX = 0

# Image size (Assuming 640x480 as in the previous example, 
# but this should match the size used during calibration!)
# We will read the size from the frame, but setting a default is good.
DEFAULT_IMG_SIZE = (640, 480) 

# Load the YOLOv8 model (YOLOv8n for fast inference)
# This step takes the most time during startup.
model = YOLO('yolov8n.pt')

# COCO dataset class ID for 'bottle'
BOTTLE_CLASS_ID = 39 

# --- FUNCTION TO LOAD AND PREPARE CALIBRATION DATA ---
def load_and_prepare_calibration(file_path):
    """
    Loads stereo parameters (K, D, R, T, Q) from the .pkl file
    and computes the necessary rectification maps (R1, P1, etc.).
    """
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)

        # *** Adjusted to match the keys in your stereo_calibration2.pkl file ***
        K1 = data['cameraMatrix1']
        D1 = data['distCoeffs1']
        K2 = data['cameraMatrix2']
        D2 = data['distCoeffs2']
        R = data['R']
        T = data['T']
        Q = data['Q'] # Q matrix is already computed

    except Exception as e:
        print(f"Error loading calibration file: {e}")
        print("Ensure the file exists and the internal keys (cameraMatrix1, R, T, Q etc.) are correct.")
        return None, None, None, None
    
    # We need the image shape from the camera, let's open them first to get the size.
    cap_test = cv2.VideoCapture(CAMERA_LEFT_INDEX)
    cap_test.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_IMG_SIZE[0])
    cap_test.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_IMG_SIZE[1])
    ret, frame = cap_test.read()
    cap_test.release()
    
    if not ret:
        print("Error: Could not read a frame to determine image size.")
        return None, None, None, None

    img_size = frame.shape[1], frame.shape[0] # (width, height)
    print(f"Using image size: {img_size}")


    # Since your calibration file did not store R1, R2, P1, P2 directly, we re-compute them 
    # using cv2.stereoRectify which only needs K, D, R, T.
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1,
        cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=img_size, R=R, T=T,
        alpha=-1 
    )

    # Create the Rectification Maps (interpolation maps)
    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)
    
    print("Stereo calibration loaded and rectification maps created successfully.")
    return (map1_l, map2_l), (map1_r, map2_r), Q, img_size


# --- FUNCTION TO CALCULATE 3D DEPTH ---
def calculate_3d_coords(disparity, x, y, Q_matrix):
    """
    Computes 3D coordinates (X, Y, Z) from a pixel (x, y) and its disparity value.
    """
    # Using the standard Reprojection formula: Q * [x, y, disparity, 1].T
    # This is a key step in stereo vision for depth estimation. 
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    
    homogeneous_point = np.dot(Q_matrix, point)
    
    w = homogeneous_point[3]
    if w == 0:
        return (0, 0, 0)
        
    X = homogeneous_point[0] / w
    Y = homogeneous_point[1] / w
    Z = homogeneous_point[2] / w
    
    return (X, Y, Z)


def get_stereo_match(results_l, results_r):
    """
    Finds a matching detection in the right frame for the best 
    detection in the left frame using the epipolar (Y-axis) constraint.
    """
    if not results_l or len(results_l[0].boxes) == 0:
        return None, None
    
    box_l = results_l[0].boxes[0]
    x_l, y_l = int(box_l.xywh[0][0].item()), int(box_l.xywh[0][1].item())
    
    for box_r in results_r[0].boxes:
        x_r, y_r = int(box_r.xywh[0][0].item()), int(box_r.xywh[0][1].item())
        # Check if they are on the same horizontal line
        if abs(y_l - y_r) < 10: 
            return (x_l, y_l), x_r
            
    return None, None


def process_depth(coords_l, x_r, Q_matrix):
    """Calculates 3D coordinates and returns the formatted text."""
    x_l, y_l = coords_l
    disparity = abs(x_l - x_r)
    
    if disparity > 0:
        X, Y, Z = calculate_3d_coords(disparity, x_l, y_l, Q_matrix)
        return X, Y, Z, f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: {Z:.1f}cm"
    return None, None, None, ""



def get_bottle_distance():
    """
    Returns:
        Z (float): distance to bottle (same units as calibration, e.g. cm)
        None: if bottle not detected
    """

    ret_l, frame_l_orig = cap_l.read()
    ret_r, frame_r_orig = cap_r.read()

    if not ret_l or not ret_r:
        return None

    frame_l_rect = cv2.remap(frame_l_orig, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
    frame_r_rect = cv2.remap(frame_r_orig, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

    results_l = model.predict(frame_l_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)
    results_r = model.predict(frame_r_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)

    if not results_l or len(results_l[0].boxes) == 0:
        return None

    box_l = results_l[0].boxes[0]
    x_l, y_l = int(box_l.xywh[0][0]), int(box_l.xywh[0][1])

    for box_r in results_r[0].boxes:
        x_r, y_r = int(box_r.xywh[0][0]), int(box_r.xywh[0][1])
        if abs(y_l - y_r) < 10:
            disparity = abs(x_l - x_r)
            if disparity > 0:
                _, _, Z = calculate_3d_coords(disparity, x_l, y_l, Q_matrix)
                return Z

    return None


def depth_to_motor_time(distance_cm, speed_cm_per_s):
    """
    distance_m: bottle dpeth in cm
    speed_cm_per_s: speed of the motor in cm/s
    """
    time_sec = distance_cm / speed_cm_per_s
    return time_sec
