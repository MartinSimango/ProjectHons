#script to capature chessboard pictures for camera calibration 
#it then calibrates and saves camera matrix and distortion coefficients in a file
#Martin Simango
import cv2
import numpy as np
import sys #for getting camera name
import os  #for making directory
import calibrateFunctions as cf



#open cameras

cap_1= cv2.VideoCapture(0)
cap_2= cv2.VideoCapture(2)

#get camera name

camera_name_1 = sys.argv[1]
camera_name_2 = sys.argv[2]
cal_file= sys.argv[3];
camera_directory_name_1= camera_name_1+"_pics"
camera_directory_name_2 = camera_name_2+"_pics"



count=0 #number of frames captured
savedImages_1=[]
savedImages_2=[]

#create folder and save the images in a folder
try:  
    #create and change to the directory
    os.mkdir(camera_directory_name_1)
except OSError:  
    print ("Creation of the directory %s failed" % camera_directory_name_1)
    exit(1)
try:
    os.mkdir(camera_directory_name_2)
except OSError:  
    print ("Creation of the directory %s failed" % camera_directory_name_2)
    exit(1)
#time to calibrate
while(True):
    #capture frame
    _,frame_1= cap_1.read()
    _,frame_2= cap_2.read()

    #try find the chess board 
    found_1=False
    found_2=False
    found_1, foundCorners_1= cv2.findChessboardCorners(frame_1,cf.CHESSBOARDSIZE,flags=cv2.CALIB_CB_ADAPTIVE_THRESH| cv2.CALIB_CB_NORMALIZE_IMAGE)
    found_2, foundCorners_2= cv2.findChessboardCorners(frame_2,cf.CHESSBOARDSIZE,flags=cv2.CALIB_CB_ADAPTIVE_THRESH| cv2.CALIB_CB_NORMALIZE_IMAGE) 
    #copy the frame to draw on
    foundFrame_1=np.copy(frame_1)
    foundFrame_2=np.copy(frame_2)
    if(found_1):
        cv2.drawChessboardCorners(foundFrame_1,cf.CHESSBOARDSIZE,foundCorners_1,found_1)
        cv2.imshow("Frame_1",foundFrame_1)
    else:
        cv2.imshow("Frame_1",frame_1)
    if(found_2):
        cv2.drawChessboardCorners(foundFrame_2,cf.CHESSBOARDSIZE,foundCorners_2,found_2)
        cv2.imshow("Frame_2",foundFrame_2)
    else:
        cv2.imshow("Frame_2",frame_2)


    character= cv2.waitKey(60)&0xff
    #save image (space)
    if(character==32):
        if(found_1 and found_2):
            count+=1
            savedImages_1.append(frame_1)
            savedImages_2.append(frame_2)
            print(count,"pictures taken")
    if(character==ord('q')):
        break

if(len(savedImages_1)<15):
    print("Not enough images for calibration")
    os.rmdir(camera_directory_name_1)
    os.rmdir(camera_directory_name_2)
    exit(1)


    #save new pictures
os.chdir(camera_directory_name_1) 
print()
print("Now saving files for camera_1")
for i in range(0,len(savedImages_1)):
    cv2.imwrite(camera_name_1+"_"+str(i)+".png",savedImages_1[i])
print(len(savedImages_1),"Images saved")
print()
os.chdir("../"+camera_directory_name_2)
print()
print("Now saving files for camera_2")
for i in range(0,len(savedImages_2)):
    cv2.imwrite(camera_name_2+"_"+str(i)+".png",savedImages_2[i])
print(len(savedImages_2),"Images saved")
print()

M1, D1,objPoints_1,imgPoints_1,rvecs_1,tvecs_1=cf.cameraCalibration(savedImages_1,cf.CHESSBOARDSIZE,cf.SQUARESIZE)# cameraCalibration(
M2, D2,objPoints_2,imgPoints_2,rvecs_2,tvecs_2=cf.cameraCalibration(savedImages_2,cf.CHESSBOARDSIZE,cf.SQUARESIZE)# cameraCalibration(


ret,M1,d1,M2,d2,R,T,E,F=cf.stereoCalibration(objPoints_1,imgPoints_1,imgPoints_2,M1,D1,M2,D2,savedImages_1[0])
print("Value of ret was: ",ret)
os.chdir("../") #go back to the main directory


params=[]
params.append(M1)
params.append(d1)
params.append(M2)
params.append(d2)
params.append(R)
params.append(T)
params.append(E)
params.append(F)
#M1 d1 M2 d2 R T E F
if(not cf.saveStereoCameraCalibration(params,cal_file)):
    print("Failed to save calibration file!")
    exit(1)

print("Saved Calibration to "+cal_file+".cal")

#close cameras
cap_1.release() 
cap_2.release()
cv2.destroyAllWindows() 

