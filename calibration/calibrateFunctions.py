
import cv2
import numpy as np
import glob #for reading in images from folder

#setup constants for camera calibration
SQUARESIZE=0.022 #meters (length of square side)
CHESSBOARDSIZE= (6,9)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


#read in images from a folder name
def readImages(folder_name):
        images = [cv2.imread(file) for file in glob.glob(folder_name+"/*.png")]
        return images

#create the known board positions for each picture given the board size and edgeLength of the squares
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) 
def createKnownBoardPositions(size, edgeLength,cornerPositions):
    width,height= size
    for i in range(0,height):
        for j in range(0,width):
            cornerPositions.append([j*edgeLength,i*edgeLength,0.0]) 
    

#get the board positions 
def getChessBoardCorners(images,allFoundCorners,showResults):
    for image in images:
        found,corners= cv2.findChessboardCorners(image,CHESSBOARDSIZE,flags=cv2.CALIB_CB_ADAPTIVE_THRESH| cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if(found):

                cv2.cornerSubPix(image,corners,(11,11),(-1,-1),criteria)
                allFoundCorners.append(corners)
        if(showResults):
                cv2.drawChessboardCorners(image,CHESSBOARDSIZE,corners,found)
                cv2.imshow('corners',image)
                cv2.waitKey(0)


def cameraCalibration(images,boardSize,edgeLength):
        boardImageSpacePoints=[]
        getChessBoardCorners(images,boardImageSpacePoints,False) #stores 2d board points of every image
        worldSpacePoints=[[]] #empty array of arrays
        createKnownBoardPositions(boardSize,edgeLength,worldSpacePoints[0]) 
        wSP=np.array(worldSpacePoints[0],dtype=np.float32)
        #repeat array with the number of image times
        for i in range(1,len(boardImageSpacePoints)):
                worldSpacePoints.append(wSP)
        worldSpacePoints=np.array(worldSpacePoints,dtype=np.float32)

        #distortionCoefficients=np.zeros((8,1),dtype=cv2.CV_64F) 
        #camera matrix, distance coefficients, radial vectors and tangential vectors
        ret, cameraMatrix, distortionCoefficients, rvecs, tvecs = cv2.calibrateCamera(worldSpacePoints,boardImageSpacePoints,images[0].shape[::-1], None, None)
        return cameraMatrix, distortionCoefficients,worldSpacePoints,boardImageSpacePoints,rvecs,tvecs

#from https://github.com/bvnayak/stereo_calibration/blob/master/camera_calibrate.py
def stereoCalibration(objPoints,img1_points,img2_points,camM1,camD1,camM2,camD2,image):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        #ret,M1,d1,M2,d2,R,T,E,F
        #size = image.shp
        return cv2.stereoCalibrate(objPoints,img1_points,img2_points,camM1,camD1,camM2,camD2,image.shape[::-1],criteria=stereocalib_criteria,flags=flags)
#EDIT
def saveCameraCalibration(cameraMatrix,distortionCoefficients,filename):  
        f= open(filename+".cal","w") #open file for writing
        if(f.closed):
                print("Failed to open file:",filename)
                return False;
        #write camera matrix
        rows= len(cameraMatrix)
        cols= len(cameraMatrix[0])
        f.write(str(rows)+","+str(cols)+"\n")
        for r in range(0,rows):
                for c in range(0,cols):
                        val=cameraMatrix[r][c]
                        f.write(str(val)+"\n")

        #write distance matrix
        rows= len(distortionCoefficients)
        cols= len(distortionCoefficients[0])
        f.write(str(rows)+","+str(cols)+"\n")
        for r in range(0,rows):
                for c in range(0,cols):
                        val=distortionCoefficients[r][c]
                        f.write(str(val)+"\n")

        f.close()
        return True

#returns camera matrix and distance coefficients 
def loadCameraCalibration(filename):
        f= open(filename,"r")
        if(f.closed):
                print("Failed to open file:",filename)
                return [],[];
        #load the camera matrix
       
        rows,cols=map(int,f.readline().split(","))

        cameraMatrix=[[0 for x in range(cols)] for y in range(rows)] 
        for r in range(0,rows):
                for c in range(0,cols):
                    line=f.readline().split();
                   
                    cameraMatrix[r][c]=float(line[0]);

        
        rows,cols=map(int,f.readline().split(","))
        distortionCoefficients=[[0 for x in range(cols)] for y in range(rows)] 
       
        for r in range(0,rows):
                for c in range(cols):
                    line=f.readline().split();
                
                    distortionCoefficients[r][c]=float(line[0])
        f.close()
        return cameraMatrix,distortionCoefficients

def saveStereoCameraCalibration(params,filename):
        f= open(filename+".cal","w")
        if(f.closed):
                print("Failed to open file:",filename)
                return False;
        #write each matrix in params to a file
        f.write(str(len(params))+"\n")
        for i in range(len(params)):
                rows= len(params[i])
                cols= len(params[i][0])
                f.write(str(rows)+","+str(cols)+"\n")
                for r in range(0,rows):
                        for c in range(0,cols):
                                val= params[i][r][c]
                                f.write(str(val)+"\n")
                        
        f.close()
        return True

def loadStereoCameraCalibration(filename):
        f= open(filename,"r")
        if(f.closed):
                print("Failed to open file:",filename)
                return []; 
        array_len= int(f.readline().split()[0])
        ret_Matrix=[]
        for i in range(array_len):
                rows,cols=map(int,f.readline().split(","))
                tmpMatrix=[[0 for x in range(cols)] for y in range(rows)] 
                for r in range(0,rows):
                    for c in range(0,cols):
                        line=f.readline().split();
                        tmpMatrix[r][c]=float(line[0]); 
                ret_Matrix.append(tmpMatrix)
        return ret_Matrix


def undistortImage(cam_matrix,dist,image):
    h,  w = image.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_matrix,dist,(w,h),1,(w,h))
    #undistort
    new_image=cv2.undistort(image, cam_matrix, dist, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    return new_image,newcameramtx,dst


def stereoRectifiction(M1,D1,M2,D2,R,T,image):
        #R1,R2,P1,P2,Q,validPixROI1,validPIXROI2
        return cv2.stereoRectify(M1,D1,M2,D2,image.shape[::-1],R,T)

#method from http://timosam.com/python_opencv_depthimage
def getDisparity(imgL,imgR):
         # SGBM Parameters -----------------
        window_size = 5                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

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

#detect and match features in both images
def detetectAndMatch(imgL,imgR):
#TODO





