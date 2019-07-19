import cv2
import numpy as np
import calibrateFunctions as cf
import sys
import math
from sklearn.preprocessing import normalize
#takes in two camera calibration file names respective camera numbers
from matplotlib import pyplot as plt

if(len(sys.argv) !=6):
    print("Argument 1: Camera 1 calibration file")
    print("Argument 2: Camera 2 calibration file")
    print("Argument 3: Camera 1 camera number")
    print("Argument 4: Camera 2 camera number")
    print("Argument 5: Distance between 2 cameras ")
    exit(1)

cal_file_1= sys.argv[1] 
cal_file_2= sys.argv[2]

cam_1_no= int(sys.argv[3])
cam_2_no= int(sys.argv[4])

GAP= float(sys.argv[5]);
#load calibration files
cam_matrix_1,distortion_coefficients_1=cf.loadCameraCalibration(cal_file_1)
cam_matrix_2,distortion_coefficients_2=cf.loadCameraCalibration(cal_file_2)

cam_matrix_1= np.array(cam_matrix_1);
distortion_coefficients_1=np.array(distortion_coefficients_1);
cam_matrix_2= np.array(cam_matrix_2);
distortion_coefficients_2= np.array(distortion_coefficients_2);



cap_1= cv2.VideoCapture(cam_1_no)
cap_2= cv2.VideoCapture(cam_2_no)

cv2.namedWindow('cam_1',cv2.WINDOW_NORMAL)
cv2.namedWindow('cam_2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('cam_1', 600,600)
cv2.resizeWindow('cam_2',600,600)

def undistortImage(image,cam_matrix,dist):

    h,  w = image.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_matrix,dist,(w,h),1,(w,h))
    #undistort
    new_image=cv2.undistort(image, cam_matrix, dist, None, newcameramtx)
    return new_image,newcameramtx,roi

#method from http://timosam.com/python_opencv_depthimage
def getDisparity(imgL,imgR):
    # SGBM Parameters -----------------
    window_size = 3                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

    left_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=160,             # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=5,
    P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32 * 3 * window_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=0,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    # FILTER Parameters
    lmbda = 80000
    sigma = 1.2
    visual_multiplier = 1.0

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    print('computing disparity...')
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    filteredImg = np.uint8(filteredImg)
    return filteredImg;
def getDepth(disparity,new_matrix):
    fx= new_matrix[0][0]
    fy= new_matrix[1][1]
    focal_length= math.sqrt(fx**2+fy**2);
    depth = disparity.copy();
    h,w   = depth.shape[:2];
    for i in range(0,h):
        for j in range(0,w):
            if(disparity[i][j]==0):
                depth[i][j]=255;
            else:
                depth[i][j] = (GAP * focal_length)/disparity[i][j];

    return depth;

while(True):
    _,frame_1= cap_1.read()
    _,frame_2= cap_2.read()

    new_imgL,new_matrix_1,_=undistortImage(frame_1,cam_matrix_1,distortion_coefficients_1);
    new_imgR,new_matrix_2,_=undistortImage(frame_2,cam_matrix_2,distortion_coefficients_2);

    disparity= getDisparity(new_imgL,new_imgR);
    cv2.imshow("cam_1",new_imgL)
    cv2.imshow("cam_2",new_imgR)

    cv2.imshow("disparity",cv2.resize(disparity,(600,600)))

    depth= getDepth(disparity,new_matrix_1)
    cv2.imshow("depth",depth);

    character= cv2.waitKey(60) & 0xff
    if(character==ord('q')):
        break;
    


cap_1.release()
cap_2.release()
cv2.destroyAllWindows()
