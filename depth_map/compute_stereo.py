import numpy as np
import cv2 as cv
import calibrateFunctions as cf
import math
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
imgL = cv.imread('left.png',cv.IMREAD_GRAYSCALE)
imgR = cv.imread('right.png',cv.IMREAD_GRAYSCALE)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)

gap=10 #distance between two cameras
#focal length
fx=701
fy=326
fz = math.sqrt(fx**2+fy**2)

disparity= abs(imgR-imgL);
disparity = stereo.compute(imgL,imgR)
depth= disparity.copy();
h,w=disparity.shape[:2];
for i in range(0,h):
    for j in range(0,w):
        if(disparity[i][j]==0):
            depth[i][j]=0;
        else:
            depth[i][j] = (gap * fz)/disparity[i][j];

#now range depth from 0 to 255
min_d= np.min(depth);
max_d= np.max(depth)
print(depth)
cv.imshow("depth",cv.resize(depth,(600,600)))
cv.imshow("dif",cv.resize(disparity,(600,600)))



cv.waitKey(0)