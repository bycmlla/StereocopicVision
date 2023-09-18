import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpointsL = []  # 3d point in real world space
imgpointsL = []  # 2d points in image plane.
objpointsR = []
imgpointsR = []

images = glob.glob('Calibração/camera0/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    grayL = cv2.cvtColor(img, 0)

    ret, cornersL = cv2.findChessboardCorners(grayL, (9, 6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsL.append(objp)
        cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        imgpointsL.append(cornersL)

images = glob.glob('Calibração/camera1/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    grayR = cv2.cvtColor(img, 0)

    # Find the chess board corners
    ret, cornersR = cv2.findChessboardCorners(grayR, (9, 6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsR.append(objp)
        cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
        imgpointsR.append(cornersR)

# Stereo calibration
flags = 0
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpointsL, imgpointsL, imgpointsR, (320, 240),
    flags=flags, criteria=criteria_stereo
)

print("Stereo Calibration completed.")
print("Camera Matrix 1:\n", cameraMatrix1)
print("Distortion Coefficients 1:\n", distCoeffs1)
print("Camera Matrix 2:\n", cameraMatrix2)
print("Distortion Coefficients 2:\n", distCoeffs2)
print("Rotation Matrix:\n", R)
print("Translation Vector:\n", T)
print("Essential Matrix:\n", E)
print("Fundamental Matrix:\n", F)
