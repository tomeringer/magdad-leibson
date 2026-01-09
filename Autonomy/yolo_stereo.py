import pickle
import time
from datetime import datetime

import cv2
import numpy as np
from ultralytics import YOLO

import lib_logger


# --- FUNCTION TO LOAD AND PREPARE CALIBRATION DATA ---
def open_cam(path: str, name: str):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    print(f"{name}: opening {path}  isOpened={cap.isOpened()}")
    if not cap.isOpened():
        return cap

    # Force MJPG (compressed) to reduce USB bandwidth
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

    # Keep a small buffer; 1 can be too aggressive on some drivers
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    # Warm up
    for _ in range(10):
        cap.read()

    print(f"{name}: negotiated "
          f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.1f}fps "
          f"fourcc={int(cap.get(cv2.CAP_PROP_FOURCC))}")
    return cap


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
        Q = data['Q']  # Q matrix is already computed

    except Exception as e:
        print(f"Error loading calibration file: {e}")
        print("Ensure the file exists and the internal keys (cameraMatrix1, R, T, Q etc.) are correct.")
        return None, None, None, None

    # We need the image shape from the camera, let's open them first to get the size.
    # cap_test = cv2.VideoCapture(RIGHT, cv2.CAP_V4L2)
    # cap_test.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_IMG_SIZE[0])
    # cap_test.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_IMG_SIZE[1])
    # ret, frame = cap_test.read()
    # cap_test.release()

    # if not ret:
    #    print("Error: Could not read a frame to determine image size.")
    #    return None, None, None, None

    img_size = DEFAULT_IMG_SIZE  # (width, height)
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


if __name__ == '__main__':
    # --- CONFIGURATION ---
    ts = datetime.now().strftime("%Y%m%d_%H%M")
    metrics = lib_logger.CSVMetricLogger(
        f"cam_logs/log{ts}.csv",
        fieldnames=["t_unix", "bottle_x", "bottle_y", "bottle_z", "t_proc"]
    )

    # Path to your generated stereo calibration file
    CALIBRATION_FILE_PATH = r"stereo_calibration.pkl"

    # Indices of your USB cameras (must match the cameras used for calibration)
    LEFT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
    RIGHT = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

    # Image size (Assuming 640x480 as in the previous example,
    # but this should match the size used during calibration!)
    # We will read the size from the frame, but setting a default is good.
    W = 640
    H = 480
    CAM_FPS = 15
    DEFAULT_IMG_SIZE = (W, H)

    # Load the YOLOv8 model (YOLOv8n for fast inference)
    # This step takes the most time during startup.
    # model = YOLO('yolov8n.pt')  
    model = YOLO('yolov8n_ncnn_model', task='detect')  # Use the NCNN exported model for better performance on Pi    
    
    # COCO dataset class ID for 'bottle'
    BOTTLE_CLASS_ID = 39

    cap_l = open_cam(LEFT, "LEFT")
    cap_r = open_cam(RIGHT, "RIGHT")

    # --- MAIN LOGIC ---
    # Load and prepare calibration data, and get the image size
    maps_l, maps_r, Q_matrix, img_size = load_and_prepare_calibration(CALIBRATION_FILE_PATH)

    if maps_l is None:
        exit()

    print("Starting YOLOv8 Stereo Detection.")

    while True:
        t0 = time.perf_counter()

        ret_l, frame_l_orig = cap_l.read()
        ret_r, frame_r_orig = cap_r.read()

        if not ret_l or not ret_r:
            print("Error: Failed to read frames. Check camera connection.")
            break

        # --- 1. Rectification (Undistortion and Alignment) ---
        # This aligns the images so matching points lie on the same horizontal line.
        frame_l_rect = cv2.remap(frame_l_orig, maps_l[0], maps_l[1], cv2.INTER_LINEAR)
        frame_r_rect = cv2.remap(frame_r_orig, maps_r[0], maps_r[1], cv2.INTER_LINEAR)

        # --- 2. YOLO Detection ---
        # Run the model on both rectified frames independently for object detection.
        results_l = model.predict(frame_l_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)
        results_r = model.predict(frame_r_rect, conf=0.50, classes=[BOTTLE_CLASS_ID], verbose=False)

        #annotated_l = results_l[0].plot()
        #annotated_r = results_r[0].plot()

        # --- 3. 3D Depth Calculation (Association & Reprojection) ---

        # Process the most confident detection from the left frame
        X, Y, Z = None, None, None
        if results_l and len(results_l[0].boxes) > 0:
            box_l = results_l[0].boxes[0]
            # Get the center of the bounding box
            x_l, y_l = int(box_l.xywh[0][0].item()), int(box_l.xywh[0][1].item())

            # Simple Association: Find a close match in the right frame based on the Y coordinate
            closest_box_r = None

            for box_r in results_r[0].boxes:
                x_r, y_r = int(box_r.xywh[0][0].item()), int(box_r.xywh[0][1].item())

                # Since images are rectified, we check if Y coordinates are close (Epipolar constraint)
                if abs(y_l - y_r) < 10:
                    closest_box_r = box_r
                    break

            if closest_box_r is not None:
                x_r = int(closest_box_r.xywh[0][0].item())

                # Calculate the Disparity (horizontal distance: d = x_l - x_r)
                disparity = abs(x_l - x_r)

                if disparity > 0:
                    X, Y, Z = calculate_3d_coords(disparity, x_l, y_l, Q_matrix)

                    # Draw the 3D coordinates (Z is the depth) onto the left frame
                    # Units (mm/cm/m) depend on how the Translation vector T was calibrated.
                    text = f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: {Z:.1f}cm"
                    #cv2.putText(annotated_l, text, (x_l - 150, y_l - 40),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    print(text)

        # --- 4. Display ---
        # Concatenate the two annotated frames
        #combined_frame = np.concatenate((annotated_l, annotated_r), axis=1)

        t_total = time.perf_counter() - t0
        metrics.log(t_unix=time.perf_counter(), bottle_x=X, bottle_y=Y, bottle_z=Z, t_proc=t_total)

    # Release resources
    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()
    print("Program terminated. Resources released.")
