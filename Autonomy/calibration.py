import cv2
import numpy as np
import glob
import os

# Chessboard settings (inner corners)
chessboard_size = (6, 9)  # columns x rows of inner corners

# Prepare object points
objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2) * 1.984375  # assuming square size in cm

# Prepare lists
objpoints = []
imgpoints_left = []
imgpoints_right = []

# Load images
images_left = sorted(glob.glob("C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_images\\left_*.png"))
images_right = sorted(glob.glob("C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_images\\right_*.png"))

good_images = [] 

for left_img_path, right_img_path in zip(images_left, images_right):
    imgL = cv2.imread(left_img_path)
    imgR = cv2.imread(right_img_path)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    
    retL, cornersL = cv2.findChessboardCorners(grayL, chessboard_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboard_size, None)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    if retL:
        cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)

    if retR:
        cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)

    
    if retL and retR:
        objpoints.append(objp)
        imgpoints_left.append(cornersL)
        imgpoints_right.append(cornersR)
        good_images.append((left_img_path, right_img_path))
        print(f"Good pair: {left_img_path}, {right_img_path}")
        
        # Draw detected corners
        cv2.drawChessboardCorners(imgL, chessboard_size, cornersL, retL)
        cv2.drawChessboardCorners(imgR, chessboard_size, cornersR, retR)
        cv2.imshow("Corners Left", imgL)
        cv2.imshow("Corners Right", imgR)
        cv2.waitKey(100)
    else:
        print(f"Bad pair: {left_img_path}, {right_img_path}")

cv2.destroyAllWindows()

if len(objpoints) == 0:
    print("No good images detected. Cannot calibrate.")
    exit()

# Image size
img_shape = grayL.shape[::-1]

# Calibrate individual cameras
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpoints_left, img_shape, None, None)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpoints_right, img_shape, None, None)

# Stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    mtxL, distL, mtxR, distR,
    img_shape, criteria=(cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5),
    flags=flags
)

print("Left reprojection error:", retL)
print("Right reprojection error:", retR)
print("Stereo error:", retval)

# Rectification maps
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, img_shape, R, T)
mapLx, mapLy = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, img_shape, cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, img_shape, cv2.CV_32FC1)

# Show rectified images with horizontal lines
for left_img_path, right_img_path in good_images:
    imgL = cv2.imread(left_img_path)
    imgR = cv2.imread(right_img_path)
    rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)
    
    h, w = rectL.shape[:2]
    for y in range(0, h, 50):
        cv2.line(rectL, (0, y), (w, y), (0, 255, 0), 1)
        cv2.line(rectR, (0, y), (w, y), (0, 255, 0), 1)
    
    cv2.imshow("Rectified Left", rectL)
    cv2.imshow("Rectified Right", rectR)
    cv2.waitKey(500)  # show each pair for 0.5 sec

cv2.destroyAllWindows()
print("All rectified images displayed.")

import pickle

calib_data = {
    "cameraMatrix1": cameraMatrix1,
    "distCoeffs1": distCoeffs1,
    "cameraMatrix2": cameraMatrix2,
    "distCoeffs2": distCoeffs2,
    "R": R,
    "T": T,
    "Q": Q
}


with open("C:\\Users\\TLP-001\\Desktop\\magdad-leibson\\Autonomy\\stereo_calibration.pkl", "wb") as f:
    pickle.dump(calib_data, f)

print("Calibration data saved to stereo_calibration.pkl")


# Create stereo matcher
stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

# Example on first good pair
imgL = cv2.imread(good_images[0][0], cv2.IMREAD_GRAYSCALE)
imgR = cv2.imread(good_images[0][1], cv2.IMREAD_GRAYSCALE)

rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)

disparity = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

fx_left = cameraMatrix1[0,0]
fy_left = cameraMatrix1[1,1]

fx_right = cameraMatrix2[0,0]
fy_right = cameraMatrix2[1,1]

print(f"Left camera fx, fy: {fx_left:.2f}, {fy_left:.2f}")
print(f"Right camera fx, fy: {fx_right:.2f}, {fy_right:.2f}")

cv2.imshow("Disparity", (disparity - disparity.min()) / (disparity.max() - disparity.min()))
cv2.waitKey(0)
cv2.destroyAllWindows()
