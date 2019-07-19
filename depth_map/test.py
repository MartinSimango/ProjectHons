import numpy as np
from sklearn.preprocessing import normalize
import cv2
import math

cap_1= cv2.VideoCapture(2)


while(1):
    _,frame_1= cap_1.read()

    cv2.imshow("depth",frame_1)

    character= cv2.waitKey(60) & 0xff
    if(character==ord('q')):
        break;
    


