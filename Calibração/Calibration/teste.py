import numpy as np
import cv2
import glob

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*9, 3), np.float32)

objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

objpointsL = []  # 3d point in real world space
imgpointsL = []  # 2d points in image plane.
objpointsR = []
imgpointsR = []

images = glob.glob('left*.jpg')

for fname in images:
    img = cv2.imread(fname)
    grayL = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, cornersL = cv2.findChessboardCorners(grayL, (9, 6), None)

    if ret == True:
        objpointsL.append(objp)
        cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        imgpointsL.append(cornersL)

images = glob.glob('right*.jpg')

for fname in images:
    img = cv2.imread(fname)
    grayR = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, cornersR = cv2.findChessboardCorners(grayR, (9, 6), None)

    if ret == True:
        objpointsR.append(objp)
        cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
        imgpointsR.append(cornersR)

retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpointsL, imgpointsL, imgpointsR, (320, 240))
