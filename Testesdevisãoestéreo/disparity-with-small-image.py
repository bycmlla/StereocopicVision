import numpy as np
import cv2

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

stereo = cv2.StereoSGBM_create(
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
imgL = cv2.imread("color1_small.jpg")
imgR = cv2.imread("color2_small.jpg")

disparity = stereo.compute(imgL, imgR)
if disparity is not None:
    disparity = disparity.astype(np.float32) / 16.0
else:
    print("A computação da disparidade falhou.")


cv2.imshow('leftview',imgL)
cv2.imshow('rightview', imgR)
cv2.imshow('disparity', (disparity - minDisparity) / numDisparities)
cv2.waitKey()