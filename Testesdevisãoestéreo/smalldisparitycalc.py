import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
imgL = cv.imread('/home/aluno/Downloads/stereoscopic/color1_small.jpg',0)
imgR = cv.imread('/home/aluno/Downloads/stereoscopic/color2_small.jpg',0)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=21)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()