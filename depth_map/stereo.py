import cv2
import numpy as np
import calibrateFunctions

#undistor the images

cap_1= cv2.VideoCapture(1)
cap_2= cv2.VideoCapture(2)
cv2.namedWindow('cam_1',cv2.WINDOW_NORMAL)
cv2.namedWindow('cam_2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('cam_1', 600,600)
cv2.resizeWindow('cam_2',600,600)
while(True):
    _,frame_1= cap_1.read()
    _,frame_2= cap_2.read()
    cv2.imshow("cam_1",frame_1)
    cv2.imshow("cam_2",frame_2)
    character= cv2.waitKey(60) & 0xff
    if(character==ord('q')):
        break;
    if(character==32):
        cv2.imwrite("left.png",frame_2)
        cv2.imwrite("right.png",frame_1)


cap_1.release()
cap_2.release()
cv2.destroyAllWindows()
