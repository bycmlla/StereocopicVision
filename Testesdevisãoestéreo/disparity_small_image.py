import numpy as np
import cv2 as cv

minDisparity = 1
numDisparities = 60 - minDisparity
blockSize = 5
uniquenessRatio = 1
speckleWindowSize = 3
speckleRange = 3
disp12MaxDiff = 100
width = 640
baseline = 0.6
P1 = 600
P2 = 2400

stereo = cv.StereoSGBM_create(
    minDisparity=minDisparity,
    numDisparities=numDisparities,
    blockSize=blockSize,
    uniquenessRatio=uniquenessRatio,
    speckleWindowSize=speckleWindowSize,
    speckleRange=speckleRange,
    disp12MaxDiff=disp12MaxDiff,
    P1=P1,
    P2=P2
)
imgL = cv.imread("color1_small.jpg")
imgR = cv.imread("color2_small.jpg")

disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

cv.imshow('leftview',imgL)
cv.imshow('rightview', imgR)
cv.imshow('disparity', (disparity - minDisparity) / numDisparities)
cv.waitKey()