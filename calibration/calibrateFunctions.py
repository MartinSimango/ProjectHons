
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
        ret, cameraMatrix, distortionCoefficients, rvecs, tvecs = cv2.calibrateCamera(worldSpacePoints,boardImageSpacePoints,boardSize, None, None)
        return cameraMatrix, distortionCoefficients


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
        cv2.stereoCalibrate()

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
        
        



#retval,cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpointsL, imgpointsL, imgpointsR, (320,240)