# -*- coding: utf-8 -*-
import cv2
import time
from ultralytics import YOLO

# --- CONFIGURATION ---
CAMERA_LEFT_INDEX = 1
CAMERA_RIGHT_INDEX = 0
# Using YOLOv8n (nano) which is the fastest version for Raspberry Pi
MODEL_NAME = 'yolov8n.pt'

def main():
    print("Initializing YOLOv8 diagnostic test...")
    
    # Load model
    # This might take 10-20 seconds on a Raspberry Pi
    model = YOLO(MODEL_NAME)
    print(f"Model {MODEL_NAME} loaded.")

    # Initialize cameras
    cap_l = cv2.VideoCapture(CAMERA_LEFT_INDEX)
    cap_r = cv2.VideoCapture(CAMERA_RIGHT_INDEX)

    # Check if cameras are opened
    if not cap_l.isOpened() or not cap_r.isOpened():
        print("Error: Could not open one or both cameras.")
        return

    print("Cameras started. Press 'q' to quit.")

    # Track FPS to see performance on the Pi
    prev_time = 0

    try:
        while True:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()

            if not ret_l or not ret_r:
                print("Failed to grab frames.")
                break

            # Run YOLO inference on both frames
            # verbose=False keeps the console clean
            results_l = model.predict(frame_l, conf=0.5, verbose=False)
            results_r = model.predict(frame_r, conf=0.5, verbose=False)

            # Annotate frames with detection results
            annotated_l = results_l[0].plot()
            annotated_r = results_r[0].plot()

            # Calculate FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Combine frames side-by-side for display
            combined = cv2.hconcat([annotated_l, annotated_r])
            
            # Draw FPS on the screen
            cv2.putText(combined, f"FPS: {fps:.2f}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Stereo Test", combined)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Cleanup
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print("Cameras released and windows closed.")

if __name__ == "__main__":
    main()