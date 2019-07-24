import cv2
import numpy as np
import calibrateFunctions as cf
import sys
import math
from cv2 import aruco
from sklearn.preprocessing import normalize
#takes in two camera calibration file names respective camera numbers
from matplotlib import pyplot as plt

if(len(sys.argv) !=5):
    print("Argument 1: Camera 1 camera number")
    print("Argument 2: Camera 2 camera number")
    print("Argument 3: Calibration file")
    print("Argument 4: Distance between 2 cameras ")
    exit(1)


arucoSquare=0.079
cam_1_no= int(sys.argv[1])
cam_2_no= int(sys.argv[2])

cal_file= sys.argv[3] 
GAP= float(sys.argv[4]);
#load calibration files
#M1 d1 M2 d2 R T E F
Params= cf.loadStereoCameraCalibration(cal_file)
M1= np.array(Params[0])
d1=np.array(Params[1])
M2=np.array(Params[2])
d2=np.array(Params[3])
R=np.array(Params[4])
T=np.array(Params[5])
E=np.array(Params[6])
F=np.array(Params[7])



cap_1= cv2.VideoCapture(cam_1_no)
cap_2= cv2.VideoCapture(cam_2_no)

cv2.namedWindow('cam_1',cv2.WINDOW_NORMAL)
cv2.namedWindow('cam_2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('cam_1', 600,600)
cv2.resizeWindow('cam_2',600,600)

    

sift =  cv2.xfeatures2d.SIFT_create() 

FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

def distance_to_camera(knownWidth,focalLength,perWidth):
        return (knownWidth * focalLength)/ perWidth
def getDepth(disparity,new_matrix):
    fx= new_matrix[0][0]
    fy= new_matrix[1][1]
    focal_length= 510.7594; #pre computed  
    h,w   = disparity.shape[:2];
    depth=[[0 for x in range(w)] for y in range(h)] 
    for i in range(0,h):
        for j in range(0,w):
            if(disparity[i][j]==0):
                depth[i][j]=1000;
            else:
                depth[i][j]=(GAP * focal_length)/disparity[i][j];

    return depth;
KNOWN_WIDTH=7.9
KNOWN_DISTANCE=15
FOCAL= 510.7594 # 527.848
prevFrame_1=[];
prevFrame_2=[]

def getMatches(pic_1,pic2,numberOfMatches):
    kp_1,des_1= sift.detectAndCompute(frame_1,None)
    kp_2, des_2 = sift.detectAndCompute(frame_2,None)

    #brute force match


    matches = flann.knnMatch(des_1, des_2, k=2)

# store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    
    good= sorted(good,key=lambda x:x.distance)
    if(len(good)<numberOfMatches):
        numberOfMatches=len(good)
    return cv2.drawMatches(left,kp_1,right,kp_2,good[:numberOfMatches],None,flags=2)
    
while(True):
   
    _,frame_1= cap_1.read()
    _,frame_2= cap_2.read()
    #copy the previous frame
   

    #undistort images
    #new_imgL, NM1,_= cf.undistortImage(M1,d1,frame_1)
    #new_imgR, NM2,_= cf.undistortImage(M2,d2,frame_2)
  
    #rectify and undistort images
    gray_1= cv2.cvtColor(frame_1,cv2.COLOR_BGR2GRAY)
    gray_2= cv2.cvtColor(frame_2,cv2.COLOR_BGR2GRAY)
    
    left,right=cf.stereoUndistortRectifyMap(M1,M2,d1,d2,R,T,gray_1,gray_2)
  
    if(len(prevFrame_1)!=0):
        matches_l=getMatches(prevFrame_1,left,20)
        matches_r= getMatches(prevFrame_2,right,20)
        print("here")
        print(matches_l)
   
    

    #cv2.imshow("results",result)
    #compute disparity map
   
    disparity= cf.getDisparity(left,right)
    cv2.imshow("cam_1",left)
    cv2.imshow("cam_2",right)
    #if(count % 10==0):
    cv2.imshow("disparity",cv2.resize(disparity,(600,600)))
    #if(count % 10==0):
 
        #cv2.imshow("depth",cv2.resize(depth,(600,600)))
    
    markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    markerCorners,markerID, rejectedCandidates= aruco.detectMarkers(left,markerDictionary)
    
    
    r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquare,M1,d1)
    if(markerID is not None):
        #depth= getDepth(disparity,NM1)
        for i in range(len(markerID)):
            #aruco.draw
            aruco.drawAxis(left,M1,d1,r_vect[i],t_vect[i],0.1)
            aruco.drawDetectedMarkers(left,markerCorners)
            # print("Corners",markerCorners[0][0])
            #focalLength = (abs(markerCorners[0][0][1][0]-markerCorners[0][0][2][0]) * KNOWN_DISTANCE) / KNOWN_WIDTH
            #print("Focal length: ",focalLength)
            print("Length: ",distance_to_camera(KNOWN_WIDTH,FOCAL,abs(markerCorners[0][0][1][0]-markerCorners[0][0][2][0])))
            x=int(markerCorners[0][0][1][0])
            y=int(markerCorners[0][0][1][1])
          #  print("Depth: ",depth[x][y])
    cv2.imshow('Webcam',left)


    character= cv2.waitKey(10) & 0xff
    if(character==ord('q')):
        break;
    if(character==32): #pause
        cv2.waitKey(0)
    if(len(prevFrame_1)==0):
        prevFrame_1= left.copy()
        prevFrame_2= right.copy()


cap_1.release()
cap_2.release()
cv2.destroyAllWindows()
