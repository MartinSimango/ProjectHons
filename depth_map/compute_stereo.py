import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
imgL = cv.imread('left.png',0)
imgR = cv.imread('right.png',0)
stereo = cv.StereoBM_create(numDisparities=16*5, blockSize=25)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()