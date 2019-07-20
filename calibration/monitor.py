import cv2
import numpy as np
import calibrateFunctions as cf
import sys
from cv2 import aruco




cal_file_1= sys.argv[1] 


cam_matrix_1,distortion_coefficients_1=cf.loadCameraCalibration(cal_file_1)

distortion_coefficients_1=np.array(distortion_coefficients_1)

cam_matrix_1= np.array(cam_matrix_1)

cam_1_no= int(sys.argv[2])

arucoSquare=0.079

KNOWN_WIDTH=7.9
KNOWN_DISTANCE=15
FOCAL= 510.7594 # 527.848


def distance_to_camera(knownWidth,focalLength,perWidth):
        return (knownWidth * focalLength)/ perWidth

def undistortImage(image,cam_matrix,dist):

    h,  w = image.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_matrix,dist,(w,h),1,(w,h))
    #undistort
    new_image=cv2.undistort(image, cam_matrix, dist, None, newcameramtx)
    return new_image,newcameramtx,roi


def startWebCam(cameraMatrix, distCoeffs,arucoSquareDimension):
    

  
    markerCorners=[]

    #markerCorners=[]
    markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    cap = cv2.VideoCapture(cam_1_no)

    while(1):
        _,frame = cap.read()
        frame,new_matrix_1,_=undistortImage(frame,cam_matrix_1,distortion_coefficients_1)
        markerCorners,markerID, rejectedCandidates= aruco.detectMarkers(frame,markerDictionary)
       
       
        r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquareDimension,cameraMatrix,distCoeffs)
        if(markerID is not None):
            for i in range(len(markerID)):
                #aruco.draw
                aruco.drawAxis(frame,cameraMatrix,distCoeffs,r_vect[i],t_vect[i],0.1)
                aruco.drawDetectedMarkers(frame,markerCorners)
               # print("Corners",markerCorners[0][0])
                #focalLength = (abs(markerCorners[0][0][1][0]-markerCorners[0][0][2][0]) * KNOWN_DISTANCE) / KNOWN_WIDTH
                #print("Focal length: ",focalLength)
                print("Length: ",distance_to_camera(KNOWN_WIDTH,FOCAL,abs(markerCorners[0][0][1][0]-markerCorners[0][0][2][0])))
        cv2.imshow('Webcam',frame)
        if(cv2.waitKey(30)>=0): 
            break
    return 1

startWebCam(cam_matrix_1,distortion_coefficients_1,arucoSquare)