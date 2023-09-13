import numpy as np
import cv2 as cv
import glob

#####find chessboard corners - objecr points and image points

chessboardSize = (8,6)
frameSize = (480, 640)

#termination criteria

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


#prepare objetcs points

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * 24
print(objp)

# arrays to store object points and image points from all the images

objpoints = [] #3d point in the real world space
imgpointsL = [] # 2d points in image plane
imgpointsR = [] #2d points in image plane

imagesLeft = glob.glob('calibration/camera1/*.jpg')
# print(imagesLeft)
imagesRight = glob.glob('calibration/camera0/*.jpg')


for imgLeft, imgRight in zip(imagesLeft, imagesRight):
    imgL = cv.imread(imgLeft)
    imgR = cv.imread(imgRight)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    #find the chessboard corners
    retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

    #if found, add object points, images points (after refining them)

    if retL and retR == True:

        objpoints.append(objp)

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        #Draw and display the coorners

        cv.drawChessboardCorners(grayL, chessboardSize, cornersL, retL)
        cv.imshow('img left', grayL)
        cv.drawChessboardCorners(grayR, chessboardSize, cornersR, retR)
        cv.imshow('img right', grayR)
        cv.waitKey(1000)

cv.destroyAllWindows()

####################### CALIIBRATION 

retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
if not retL:
    raise ValueError("Calibration failed for the left camera.")
heightL, widthL, channelsL = imgL.shape

newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
if not retL:
    raise ValueError("Calibration failed for the right camera.")
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))

############ STEREO VISION CALIBRATION #######
flags = 0
flags |= cv.CALIB_FIX_INTRINSIC

#here we fix the intrisic camara matrixes so that only rot, trns, emat and fmat are calculated
#hence intriscic parameters are the same

criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#this step is performed to transformation between the two cameras and calculate essential and fundamental matrix

retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

########## Stereo Rectification #################################################

rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)


# Criar um dicionário com os dados dos mapas de retificação estéreo
stereoMapL_x = np.array(stereoMapL[0])
stereoMapL_y = np.array(stereoMapL[1])
stereoMapR_x = np.array(stereoMapR[0])
stereoMapR_y = np.array(stereoMapR[1])

data = {
    'stereoMapL_x': stereoMapL_x,
    'stereoMapL_y': stereoMapL_y,
    'stereoMapR_x': stereoMapR_x,
    'stereoMapR_y': stereoMapR_y
}
np.save('stereoMapL_x.npy', stereoMapL[0])
np.save('stereoMapL_y.npy', stereoMapL[1])
np.save('stereoMapR_x.npy', stereoMapR[0])
np.save('stereoMapR_y.npy', stereoMapR[1])
