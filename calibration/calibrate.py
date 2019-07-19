#script to capature chessboard pictures for camera calibration 
#it then calibrates and saves camera matrix and distortion coefficients in a file
#Martin Simango
import cv2
import numpy as np
import sys #for getting camera name
import os  #for making directory
import calibrateFunctions as cf



#open camera

cap= cv2.VideoCapture(0)

#get camera name

camera_name= sys.argv[1]

camera_directory_name= camera_name+"_pics"




count=0 #number of frames captured
savedImages=[]

if(len(sys.argv)>2): #then load pictures from folder and dont take new ones
    savedImages=cf.readImages(camera_directory_name)
else: #else take new pictures
    #create folder and save the images in a folder
    try:  
        #create and change to the directory
        os.mkdir(camera_directory_name)
    except OSError:  
        print ("Creation of the directory %s failed" % camera_directory_name)
        exit(1)

    while(True):
        #capture frame
        _,frame= cap.read()

        found=False
        found, foundCorners= cv2.findChessboardCorners(frame,cf.CHESSBOARDSIZE,flags=cv2.CALIB_CB_ADAPTIVE_THRESH| cv2.CALIB_CB_NORMALIZE_IMAGE)
        #copy the frame to draw on
        foundFrame=np.copy(frame)
        if(found):
            cv2.drawChessboardCorners(foundFrame,cf.CHESSBOARDSIZE,foundCorners,found)
            cv2.imshow("Frame",foundFrame)
        else:
            cv2.imshow("Frame",frame)

        character= cv2.waitKey(60)&0xff
        #save image (space)
        if(character==32):
            if(found):
                count+=1
                savedImages.append(frame)
                print(count,"pictures taken")
        if(character==ord('q')):
            break
   
if(len(savedImages)<15):
    print("Not enough images for calibration")
    os.rmdir(camera_directory_name)
    exit(1)

if(len(sys.argv)<3):
    #save new pictures
    os.chdir(camera_directory_name) 
    print()
    print("Now saving files")
    for i in range(0,len(savedImages)):
        cv2.imwrite(camera_name+"_"+str(i)+".png",savedImages[i])
    print(len(savedImages),"Images saved")
print()
#save calibration file
cameraMatrix, distortionCoefficients=cf.cameraCalibration(savedImages,cf.CHESSBOARDSIZE,cf.SQUARESIZE)# cameraCalibration(
if(not cf.saveCameraCalibration(cameraMatrix,distortionCoefficients,camera_name)):
    print("Failed to save calibration file!")


cap.release() #close camera
cv2.destroyAllWindows() 

