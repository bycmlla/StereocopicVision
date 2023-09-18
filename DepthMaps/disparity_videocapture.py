import numpy as np
import cv2 as cv

capR = cv.VideoCapture("nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1' ! nvvidconv flip-method=0 ! videoconvert ! appsink")
capL = cv.VideoCapture("nvarguscamerasrc sensor-id=1 ! 'video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1' ! nvvidconv flip-method=0 ! videoconvert ! appsink")

block_size = 5
num_disparities = 15

stereo = cv.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if not retL or not retR:
        break

    grayL = cv.cvtColor(frameL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(frameR, cv.COLOR_BGR2GRAY)

    disparity = stereo.compute(grayL, grayR)

    normalized_disparity = cv.normalize(disparity, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
    cv.imshow('disparity', normalized_disparity)

    if cv.waitKey(1) == 27:
        break

capL.release()
capR.release()
cv.destroyAllWindows()
