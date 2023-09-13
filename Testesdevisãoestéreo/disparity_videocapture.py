import numpy as np
import cv2 as cv

capR = cv.VideoCapture("DISPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1' ! nvoverlaysink -e")
capL = cv.VideoCapture("DISPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! 'video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1' ! nvoverlaysink -e")

capL.set(cv.CAP_PROP_FRAME_WIDTH, 640)
capL.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
capR.set(cv.CAP_PROP_FRAME_WIDTH, 640)
capR.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# set the block size and number of disparity levels
block_size = 5
num_disparities = 15

# create the stereo object
stereo = cv.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)

while True:
    # capture a frame from each camera
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    # rectify the images
    grayL = cv.cvtColor(frameL, 0)
    grayR = cv.cvtColor(frameR, 0)

    # compute the disparity map
    disparity = stereo.compute(grayL, grayR)

    # normalize and display the disparity map
    normalized_disparity = cv.normalize(disparity, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
    cv.imshow('disparity', normalized_disparity)

    # exit on escape key
    if cv.waitKey(1) == 27:
        break

# release the cameras and close the windows
capL.release()
capR.release()
cv.destroyAllWindows()
