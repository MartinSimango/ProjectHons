import calibrateFunctions as cf
import sys

camera_name_1 = sys.argv[1]
camera_name_2 = sys.argv[2]
cal_file= sys.argv[3]
camera_directory_name_1= camera_name_1+"_pics"
camera_directory_name_2 = camera_name_2+"_pics"


savedImages_1=cf.readImages(camera_directory_name_1)
savedImages_2= cf.readImages(camera_directory_name_2)


M1, D1,objPoints_1,imgPoints_1,rvecs_1,tvecs_1=cf.cameraCalibration(savedImages_1,cf.CHESSBOARDSIZE,cf.SQUARESIZE)# cameraCalibration(
M2, D2,objPoints_2,imgPoints_2,rvecs_2,tvecs_2=cf.cameraCalibration(savedImages_2,cf.CHESSBOARDSIZE,cf.SQUARESIZE)# cameraCalibration(


ret,M1,d1,M2,d2,R,T,E,F=cf.stereoCalibration(objPoints_1,imgPoints_1,imgPoints_2,M1,D1,M2,D2,savedImages_1[0])
print("Value of ret was: ",ret)

params=[]
params.append(M1)
params.append(d1)
params.append(M2)
params.append(d2)
params.append(R)
params.append(T)
params.append(E)
params.append(F)

if(not cf.saveStereoCameraCalibration(params,cal_file)):
    print("Failed to save calibration file!")
    exit(1)
print("Saved Calibration to "+cal_file+".cal")