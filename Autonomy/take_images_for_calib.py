import cv2
import os

# create directory to save images if it doesn't exist
save_dir = "C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_images"
os.makedirs(save_dir, exist_ok=True)

# open cameras
cap_left = cv2.VideoCapture(1)
cap_right = cv2.VideoCapture(0)

count = 0
while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()
    
    if not ret_left or not ret_right:
        print("Error reading cameras")
        break

    # Get timestamp of each frame
    time_left = cap_left.get(cv2.CAP_PROP_POS_MSEC)
    time_right = cap_right.get(cv2.CAP_PROP_POS_MSEC)

    # Synchronization: ensure difference is less than 10ms
    while abs(time_left - time_right) > 10:
        if time_left < time_right:
            ret_left, frame_left = cap_left.read()
            time_left = cap_left.get(cv2.CAP_PROP_POS_MSEC)
        else:
            ret_right, frame_right = cap_right.read()
            time_right = cap_right.get(cv2.CAP_PROP_POS_MSEC)

    # Display indication on screen: synchronized pair ready
    sync_text = "Synchronized: Press 'c' to capture"
    cv2.putText(frame_left, sync_text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(frame_right, sync_text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    # Display images for both cameras
    cv2.imshow("Left", frame_left)
    cv2.imshow("Right", frame_right)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        # Save synchronized pair of images
        left_path = os.path.join(save_dir, f"left_{count:02d}.png")
        right_path = os.path.join(save_dir, f"right_{count:02d}.png")
        cv2.imwrite(left_path, frame_left)
        cv2.imwrite(right_path, frame_right)
        print(f"Saved synchronized pair {count}")
        count += 1

    elif key == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()