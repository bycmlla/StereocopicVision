import numpy as np
import os
import cv2 as cv
from matplotlib import pyplot as plt

imgleft = '/home/aluno/Downloads/stereoscopic/lef2t.jpg'
imgright = '/home/aluno/Downloads/stereoscopic/right.jpg'

if os.path.isfile(imgleft):
    print("A imagem foi lida com sucesso!")
else:
    print("O arquivo da imagem não foi encontrado ou não pode ser lido.")

imgL = cv.imread(imgleft, 0)
imgR = cv.imread(imgright, 0)
cv.imshow("igmL", imgL)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL, imgR)
plt.imshow(disparity,'gray')
plt.show()