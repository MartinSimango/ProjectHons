
#import necessary libraries
#import pyrealsense2 as rs
#for image producing
#pi ip is 146.231.181.162


#aruco 
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

import cv2

import numpy as np
import imagezmq # for recieving images from client
import imutils
import sys # for selecting which port to use for server
import socket # for recieving information from the client about the camera i.e depth_scale camera_intrinsics and depths of keypoints
import json
import math 
import random
import time #for timing purposes
import visualOdometry as vo
import keyboard #to detect keyboard presses
import matplotlib.pyplot as plt # plotting map


import threading
import calibrateFunctions as cf
buffer_size=4096 #max size of data sent

DIRECTION_FACING='N' # 0 = North 1 = South 2= East 3 = West 
NS=3
KP_NUM=8
(VO_X,VO_Z)= (0,0)
X,Y=(0,0)
canVO= True
doVO=False
prev_image = None
prev_depth_image = None

blurs=[]
count=0
total_distance = 0
MAX_DEPTH_POINTS=50 # how many depth points we want to ask for
#create image hub and start start
PORT= "tcp://*:"+sys.argv[1]

socket_lock = threading.Lock()  #ensures that only one thread using the socket at a time


#socket for requesting camera information that is not pictures
def init_socket(port_num):
    #CREATE TCP SOCKET
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind socket
    server_socket.bind(('',port_num))
 
    #start listening
    server_socket.listen(5) 
    return server_socket
    
#initials both sockets one for port PORT and the other for port (PORT+1)
imageHub= imagezmq.ImageHub(open_port=PORT) #equivalent to sockets but for recieving images
server_socket= init_socket(int(sys.argv[1])+1) #start socket for listening to input for camera information
print("Server started!")

sock,address = server_socket.accept() #wait for connection from client
print(f"Connection from {address} recieved!")

#get camera_information
#would normally use a header for protocol but because we know what the client is sending here the is no use
data=sock.recv(buffer_size)
jsonData= json.loads(data.decode())

depth_scale   = jsonData.get("depth_scale") 
image_width   = jsonData.get("width") 
image_height  = jsonData.get("height") 
frame_rate    = jsonData.get("frame_rate") 
rate          = jsonData.get("rate")  # controls how fast should the server request client for depth information for key points
fx            = jsonData.get("fx") #602.881168321761  #value from depth.cal # 
fy            = jsonData.get("fy") #603.1732052013384 #value from depth.cal 
cx            = jsonData.get("cx")
cy            = jsonData.get("cy")
distCoeffs    = jsonData.get("distCoeffs")



print("Received: ", jsonData) #get data from camera

#aruco markers
markerCorners=[]
markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
arucoSquareDimension= 0.048 #length of side of square on aruco marker
graph_x={}#1000 by 1000 area
graph_y={}
cameraMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]) #form the camera matrix
IMAGE_CENTRE =((image_width)/2.0,image_height/2.0)

#given x and y in pixel coords get x and y coords in cm's
def calculateOriginOffset(x_p,y_p,depth_2_xy):
    #print("I",image_width)
   
    #calculate X and Y change using some trig, 
    # Width  = Z*w/Fx
    # Height = Z*y/Fy
    #fov=(float(x_p)/image_width) *np.deg2rad(69.4)
    x = ((depth_2_xy) * ( x_p - IMAGE_CENTRE[0] ))/fx
    #x = depth_2_xy * np.sin(fov)
    y = ((depth_2_xy) * ( IMAGE_CENTRE[1]- y_p ))/fy 

    return (x,y)




def getCorners(corners): #to give back TL TR BR BL corners for unordered corners 
    maxSum=0
    index=0
    for i in range(0,4):
        sum=corners[i][0]+corners[i][1] #BR will always have max sum of coords
        if(sum>maxSum):
            maxSum=sum
            index=i
    
    BR=corners[index]
    BL=corners[(index+1)%4]
    TL=corners[(index+2)%4]
    TR=corners[(index+3)%4]
    return [TL,TR,BL,BR]

def getPoints(xs,ys,NUM_POINTS,image):
    retPoints=[]

  
    for i in range(NUM_POINTS):
        if(xs[0]!=xs[1]):
            x=random.randrange(xs[0],xs[1])
        else:
            x=int((xs[0] +xs[1])/2) 
        if(ys[0]!=ys[1]):
            y=random.randrange(ys[0],ys[2])
        else:
            y=int((ys[0]+ys[2])/2)
        retPoints.append([x , y])
    cv2.circle(image,(x, y), 5, (0,255,0), -1)          #just to visualize point
    #cv2.circle(image,(xs[0], ys[0]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[1], ys[1]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[2], ys[2]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[3], ys[3]), 5, (0,255,0), -1) #just to visualize point
    #retPoints.append([int((xs[0] +xs[1])/3) , int((ys[0]+ys[2])/3)])

    cv2.imshow('Webcam',image)
    cv2.waitKey(frame_rate)
   # 
    #    retPoints.append([int(xs[0]),int(ys[0])])
        #retPoints.append([x_r,y_r])
    return retPoints

def isCornerinBox(corners,Box):
    #print("Is {0} in {1}".format(corners,Box))
    C_TL = (corners[0][0],corners[0][1])
    C_TR = (corners[1][0],corners[1][1])
    C_BL = (corners[2][0],corners[2][1])
    C_BR = (corners[3][0],corners[3][1])
    

    B_TL = (Box[0][0],Box[0][1])
    B_TR = (Box[1][0],Box[1][1])
    B_BL = (Box[2][0],Box[2][1])
    B_BR = (Box[3][0],Box[3][1])

    #check if corner square is in box square
    if(C_TL[0] >= B_TL[0] and C_TL[1] >= B_TL[1] ): #top left
        if(C_TR[0] <= B_TR[0] and C_TR[1] >= B_TR[1] ): #top right
            #if(C_BL[0] >= B_BL[0] and C_BL[1] <= B_BL[1]): #bottom left
                #if(C_BR[0] <= B_BR[0] and C_BR[1] <= B_BR[1]): # bottom right
                  
            return True
  
    return False
#https://stackoverflow.com/questions/11686720/is-there-a-numpy-builtin-to-reject-outliers-from-a-list
def reject_outliers(data, m=2):
    return data[abs(data - np.mean(data)) < m * np.std(data)]

def getDepthInfo(corners): #retrieves depth info for marker
    
    NUM_POINTS=100
    #get the y coords

    xs=[]
    ys=[]
    ys.append(corners[0][1])
    ys.append(corners[1][1])
    ys.append(corners[2][1])
    ys.append(corners[3][1])

    #get corressponding x points
    xs.append(corners[0][0])
    xs.append(corners[1][0]) #x-coords
    xs.append(corners[2][0]) # x-coords
    xs.append(corners[3][0])
    #get random points within that range
    depth=[]
    points= getPoints(xs,ys,NUM_POINTS,image)
    for i in range(len(points)):
        x= points[i][0]
        y= points[i][1]
        depth.append(depth_image[y,x])
    
    #depth = vo.getDepthFromCamera(x_send,y_send,sock,socket_lock)
    mid_x= int((xs[0] +xs[1])/2) 
    mid_y= int((ys[0]+ys[2])/2)
    #get all x points in line with mid_y
    dist  = np.array([])
    for i in range(len(depth)):
        if(int(depth[i]) != 0):
            dist = np.append(dist,[float(depth[i])*depth_scale*100])
             

    depth=0
    dist= reject_outliers(dist) # remove
    #print(dist)
    if(len(dist)!=0):
        depth= int(round(np.average(dist)))
     
    
                

    return mid_x, mid_y, depth
    
    


def cmToPixels(depth,cm):
    return fx*(cm/depth)
def addDepthToGraph(boxes,depth_to_ob,mids):
    THRESHHOLD=2
    global graph_x
    global graph_y
    #d= [None] * image_width
    
  
    
        



    for i in range(len(boxes)): #ith box is ith object
        start= boxes[i][0]
        width= boxes[i][1]
        if(DIRECTION_FACING=='S' or DIRECTION_FACING=='W'):
            start=start-width
        for x in range(start,start+width):
            #y= mids[i][1]
            #d= depth_image[y][x]*100*depth_scale #depth the that point in the box
            #x_coord,_ = calculateOriginOffset(x,y,d)
            #dist = abs(d-depth_to_ob[i])
            #if(dist>=THRESHHOLD):
            #    d=None
            #else:
            d= depth_to_ob[i]
            if(DIRECTION_FACING=='E' or DIRECTION_FACING=='W' ): 
                x,d =d,x

            if(DIRECTION_FACING=='S'):
                start=start-width
            if((x) in graph_x.keys() and  (x) in graph_x.keys()):
                if(x in graph_x[x] and d is not None and d in graph_y[x] ): #dont change the same point if it has already been plotted
                    continue

            if( x in graph_x.keys()):
                graph_x[x].append(x)
            else:
                graph_x[x] = [x]

            if( x in graph_y.keys()):
                graph_y[x].append(d)
            else:
                graph_y[x]=[d]   

            



    

def graphObjects(objects_currently_detected,object_mids): #do this within thread later 
    global image
    global graph_x
    global graph_y
    global mapped
    #graph should be scatter plot
    #look left and right from the object see where discontinuties are
    allBoxes=[]
    depths=[]
    mids=[] #middle of object 
    for i in objects_currently_detected.keys():
        if(i in mapped.keys()):
            if(DIRECTION_FACING in mapped[i]): #already mapped for that direction
                continue
        #depth_to_object_orth = objects_currently_detected[i][1][0][1] #distance to object (orth_dist) #objects_currently_detected[i][1][1]
        depth_to_object = objects_currently_detected[i][0][1]
       
        start= objects_currently_detected[i][1][0]
        width= objects_currently_detected[i][1][1]
        mid_x =object_mids[i][0]
        mid_y =object_mids[i][1]
        allBoxes.append((start,width))
        depths.append(depth_to_object)
        mids.append([mid_x,mid_y])

       
    addDepthToGraph(allBoxes,depths,mids)
       
         
        #now graph the scatter plot

       
    colors = np.array(["black", "green"])
    area = np.pi*3

    area_cam= area*2
        # Plot

    for points in graph_x.keys():  
        plt.scatter(graph_x[points], graph_y[points], s=area, c=colors[0], alpha=0.5)
    
    plt.scatter(X,Y,s=area_cam, c=colors[1], alpha=0.5)
    plt.title('Scatter ')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.ylim(-200, 200)
    plt.xlim(-200, 200)
    plt.pause(0.001) # plot non blocking
    

def boxContour(contour,sheet,color,draw):
    rect= cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box=np.int0(box)
    if(draw):
        cv2.polylines(sheet,[box],True, color, 2)
    return box
def filterDepthImage(dto): #distance to object
    global depth_image
    depth_image_tmp= depth_image.copy()
    for i in range(len(depth_image)):
        for j in range(len(depth_image[0])):
            depth_cm =int(depth_image[i][j]) * depth_scale *100
            y_height = (depth_cm * ( IMAGE_CENTRE[1] - i ) )/fy  # pixel height from the ground #236.63084547568047
            if((depth_cm>dto+2 or depth_cm < dto - 2) or y_height<-14):   #filter out ground and anything futher then dto +2cm
                depth_image_tmp[i][j]=0
            

    #normalize image then blur it to remove rough edges
    mi=np.min(depth_image)
    ma=np.max(depth_image)
    depth_image_tmp = cv2.normalize(depth_image_tmp, depth_image_tmp, mi, ma, cv2.NORM_MINMAX,dtype=cv2.CV_8UC3)
    blur = cv2.GaussianBlur(depth_image_tmp,(5,5),0)

    _, contours, _ = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # now find the contours in the image
    boxes=[]
    for contour in contours:
        #cv2.drawContours(image, contour, -1, (255, 255, 255), 3)
        boxContour(contour,blur,(255,255,255),True)
        boxes.append(boxContour(contour,image,(255,255,255),False))
    return boxes,blur
def getObjectWidth(box,depth):
    TL_x= box[0][0] 
    TR_x= box[1][0]
    start_x,_ = calculateOriginOffset(TL_x,0,depth)
    end_x,_ =  calculateOriginOffset(TR_x,0,depth)
    start_x= int(round(start_x))
    end_x =int(round(end_x))
    width = end_x- start_x
    return start_x,width,TL_x,TR_x

def tryTriangulate(num_seconds,knownDirection=-1): #try finding objects for num_seconds seconds #try finding max objects
    global image
    global blurs
    finished= False
    
    #move funcionality to tryTriangulate
    start_time = time.time()
    objects_currently_detected = {}
    object_ids_current =[]
    object_depths= {} #stores all the depths read for a particular object within the n amount of seconds
    object_mids = {}
    object_corners = {} # stores the box of aruco markers
    boxesFound=[]
    max_seen=0
    while(not finished):
        markerCorners,markerID, rejectedCandidates = aruco.detectMarkers(image,markerDictionary)
        # r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquareDimension,cameraMatrix,distCoeffs)
        # markerInfo=[] # number of identified markers
        # ID_s=[]
        if(markerID is not None):
            for i in range(len(markerID)):
                #draw the square around makers
                #aruco.drawAxis(image,cameraMatrix,distCoeffs,r_vect[i],t_vect[i],0.1)
                aruco.drawDetectedMarkers(image,markerCorners)
               
                current_id = markerID[i][0]
                correctedCorners=getCorners(markerCorners[i][0]) #correct order of corners to orientade correctly
                if( current_id in object_corners.keys()):
                    object_corners[current_id].append(correctedCorners)
                else:
                    object_corners[current_id] = [correctedCorners]
                if(len(object_corners[current_id])>max_seen):
                    max_seen=len(object_corners[current_id])

        if(time.time()-start_time >num_seconds): #time is up stop trying to triangulate and switch to something eles 
            finished = True
        
        #print("Number Objects found so far: ",len(object_depths))
        getNextFrame()
        

        #boxes,blur=filterDepthImage() #find objects boxes around objects should be returned
        #boxesFound.append(boxes)

        

    
        cv2.imshow('Webcam',image)  

       
      
        if(cv2.waitKey(frame_rate)==ord('s')): #stop looking
            break 
        #end while loop 

        # remove outliers of aruco markers found
    blurs=[]
    for i in object_corners.keys():
        if(len(object_corners[i])< (max_seen*30) //100):
                print("Outlier:, ",i,len(object_corners[i]),max_seen)
                continue
        mid_x,mid_y,dist=getDepthInfo(object_corners[i][0])

        #mid_x,mid_y,dist,box=getDepthInfo(object_corners[i][0]) # returns mid of objects in depths  and
        boxes,blur=filterDepthImage(dist) #find objects boxes around objects should be returned
        blurs.append(blur)
        #now find which box the the corners are in
        correctedBox=[]
        for j in range(len(boxes)): #find box which marker is in
            correctedBox= getCorners(boxes[j]) 
            if(isCornerinBox(object_corners[i][0],correctedBox)):
                break
        cv2.imshow('Blur'+str(len(blurs)),blur)  
        if(correctedBox==[]):
            print("Could not find box for marker!")
            continue
        start,width,mid_x,_= getObjectWidth(correctedBox,dist) #wich x_coord the object starts at
        object_mids[i]=[mid_x,mid_y]
        print("Average: ",i,dist)
        x_coord,_ = calculateOriginOffset(mid_x,mid_y,dist)
        x_coord =   int(round(x_coord))
        dist_orth = int(round(np.sqrt(np.square(dist)-np.square(x_coord))))# essentially  dist_orth = sqrt(z^2-x^2)
        pos=([x_coord,dist_orth],dist)
        
        objects_currently_detected[i] = (pos,(start,width))
        #objects_currently_detected.append((i,pos,(start,width)))
        object_ids_current.append(i)


  

    cv2.waitKey(frame_rate) #stop looking
        
        #now find which 

    #now the loop is done calculate averages of each object located 
    
        
    canTriangulate = len(objects_currently_detected) >=1
  
    return objects_currently_detected,object_ids_current,object_mids, canTriangulate      
            
def distBetweenVectors(A,B):
    #B is assummed to be further
    
    x_change= np.square(B[0]-A[0])
    z_change= np.square(B[1]-A[1])
    return np.sqrt(x_change+z_change) #dist between A and B



def calculatePossiblePosition(objects_detected,new_objects_detected,corressponding):
    id_A= corressponding[0]
    id_B= corressponding[1]
   
    
    new_A_vec = new_objects_detected[id_A][0][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[id_B][0][0]
    

    nx1, nz1= new_A_vec
    nx2, nz2= new_B_vec
    #object A and B absolute positions
    A= objects_detected[id_A][0][0]
    B= objects_detected[id_B][0][0]
    x1,z1 = A 
  

   
    
    x2,z2 = B
    A_mag=  np.linalg.norm(A)
    B_mag=  np.linalg.norm(B)

    
    new_C_vec = [0,0]

    #Depending on the direction facing t
    
  
    if(DIRECTION_FACING=='E'):
        nx1,nz1= nz1,nx1 
        nx2,nz2= nz2,nx2
    elif(DIRECTION_FACING=='W'):
        nx1,nz1= nz1,nx1 
        nx2,nz2= nz2,nx2
        
    new_A_vec = [nx1,nz1] 
    new_B_vec = [nx2,nz2]
    print("A",id_A,new_A_vec)
    print("B",id_B,new_B_vec)
   


   
    # AC= np.subtract(new_A_vec , new_C_vec)
   
    # BC = np.subtract(new_B_vec , new_C_vec)
    
    # AC_mag= np.linalg.norm(AC) 
   
    # BC_mag= np.linalg.norm(BC)

    
    # a_= 2*x1
    # b_= 2*z1
    # C_= np.square(A_mag) - np.square(AC_mag)
    # d_= 2*x2 
    # e_= 2*z2
    # F_ =np.square(B_mag) - np.square(BC_mag)
    # M_= e_-b_
    # N_= F_-C_
    # O_= d_-a_

    # P_= np.square(M_) + np.square(O_)
    # Q_= -d_*np.square(M_) - 2*N_*O_ + e_*M_*O_
    # R_= np.square(N_) - e_*M_*N_ + F_*np.square(M_)
    # if(O_==0 ): #x1==x2
    #     z3_1=z3_2=0
    #     x3_1=0
    #     x3_2=a_
    # elif(M_==0): #y1=y2
    #     x3_1=x3_2=0
    #     z3_1=0
    #     z3_2=B_mag
    # else:
    # #Px^2 + Qx +R=0
    # #apply quadrict formula to solve for possible positions for x
    #     print("SQRT",np.square(Q_)-4*P_*R_)
    #     x3_1= (-Q_ + (np.sqrt(np.square(Q_)-4*P_*R_)) ) / (2*P_)
    #     x3_2= (-Q_ - (np.sqrt(np.square(Q_)-4*P_*R_)) ) / (2*P_)
    #     z3_1= float('nan')
    #     z3_2= float('nan')
    #     if(not math.isnan(x3_1) and not math.isnan(x3_2)):
    #         x3_1= int(x3_1)
    #         x3_2= int(x3_2)
    #         z3_1= int(((N_-x3_1*O_) /M_))
    #         z3_2= int(((N_-x3_2*O_) /M_))
    #     else:
    #         return ([None,None],[None,None]) # cant get valid position
    #WILL CHANGE BASED ON DIRECTION
    width= new_objects_detected[i][1][1]
    if(DIRECTION_FACING=='S'):
        x3_1 = (x1+width) + nx1
        z3_1 = z1 + nz1
    elif(DIRECTION_FACING=='E'):
        x3_1 = (x1+width) - nx1
        z3_1 = z1 + nz1
    elif(DIRECTION_FACING=='W'):
        x3_1 = (x1+width) + nx1
        z3_1 = z1 + nz1
    elif(DIRECTION_FACING=='N'):
        x3_1 = (x1+width) + nx1
        z3_1 = z1 - nz1
   
    pos= [x3_1,z3_1]
    pos=  (pos, int(round(distBetweenVectors([x3_1,z3_1],[0,0]))) )
    print("Pos",pos)
    return pos

def calculateAbsolutePosition(objects_detected,new_objects_detected,corressponding):
    id_A= corressponding[0]
    id_B= corressponding[1]
    id_C= corressponding[2]
   
    
  
    new_A_vec = new_objects_detected[id_A][0][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[id_B][0][0]
    new_C_vec = new_objects_detected[id_C][0][0]



    #object A and B absolute positions
    A= objects_detected[id_A][0][0]
    B= objects_detected[id_B][0][0]
    C= objects_detected[id_C][0][0]
    
    x1,z1 = A 

    x2,z2 = B
  
    x3,z3 = C 
   
    if(DIRECTION_FACING=='S'):
        x1,z1= -x1,-z1 
        x2,z2= -x2,-z2
        x3,z3= -x3,-z3
    elif(DIRECTION_FACING=='E'):
        x1,z1= z1,x1 
        x2,z2= z2,x2
        x3,z3= z3,x3
    elif(DIRECTION_FACING=='W'):
        x1,z1= -z1,x1 
        x2,z2= -z2,x2
        x3,z3= -z3,x3


  
    A_mag=  np.linalg.norm(A)
    B_mag=  np.linalg.norm(B)
    C_mag=  np.linalg.norm(C)

            
    new_A_vec = [x1,z1] 
    new_B_vec = [x2,z2]
    new_C_vec = [x3,z3]
    new_D_vec = [0,0] # where camera is

    print("A",id_A,new_A_vec,)
    print("B",id_B,new_B_vec,)
    print("C",id_C,new_C_vec,)

    AD= np.subtract(new_A_vec , new_D_vec)
    BD = np.subtract(new_B_vec , new_D_vec)
    CD = np.subtract(new_C_vec , new_D_vec)

    AD_mag= np.linalg.norm(AD) 
    BD_mag= np.linalg.norm(BD)
    CD_mag= np.linalg.norm(CD)

    #form Matrix to solve problem

    M= np.matrix([[-2*x1,-2*z1],
                  [-2*x2,-2*z2],
                  [-2*x3,-2*z3]])

    S= np.array([AD_mag**2-A_mag**2,BD_mag**2-B_mag**2,CD_mag**2-C_mag**2])
    S.shape=(3,1)
        

    pos_x_z = np.linalg.lstsq(M, S,rcond=None)[0]
    pos_x = int((pos_x_z[0,0]))
    pos_z = int((pos_x_z[1,0]))
    pos=  ([pos_x,pos_z], distBetweenVectors([pos_x,pos_z],[0,0]))
    print("Abs pos ",pos) #if absolute position is known you can update VO position now
    #if(VO_X is None):
  

    return pos

def getAccuratePos(pos_1,pos_2,posFromVO):
    if(posFromVO==(None,None)):
        return (None,None)
    dist_1 = distBetweenVectors(posFromVO,pos_1)
    dist_2 = distBetweenVectors(posFromVO,pos_2)
    if( dist_1 <dist_2): #find the pos closest to the VO position
        return pos_1
    return pos_2

def findCamDirection(new_objects_detected,corresponding): #find in which direction the camera is pointing
    global relations
    global DIRECTION_FACING
    id_A= corresponding[0]
    id_B= corresponding[1]
  
   
    
  
    new_A_vec = new_objects_detected[id_A][0][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[id_B][0][0]



    #object A and B absolute positions
    #A= objects_detected[id_A][0][0]
    #B= objects_detected[id_B][0][0]

    OLD_DIRECTION=  DIRECTION_FACING
    left_right = 1
    front_behind=1
    x1,z1 = new_A_vec
    x2,z2 = new_B_vec
    if ( x1 < x2 ): 
        left_right = -1 # i is to the left of j
    if ( z1 < z2 ):  # i is behind j
        front_behind=-1
    #find A and B relation from relations map
    for i in range(len(relations[id_A])):
        for j in (relations[id_A][i]).keys():
            if(j==id_B): #j is object 
                l_r = relations[id_A][i][j][0] 
                f_b = relations[id_A][i][j][1]
                if( l_r ==1 and f_b ==1 ): # A was in front of and on the right of B
                    if(left_right ==1 and front_behind ==1):
                        #still the same direction
                        DIRECTION_FACING = DIRECTION_FACING
                    elif(left_right ==1 and front_behind ==-1):
                        DIRECTION_FACING = 'W' # now facing West
                    elif(left_right ==-1 and front_behind ==1):
                        DIRECTION_FACING = 'E'
                    elif(left_right ==-1 and front_behind ==-1):
                        DIRECTION_FACING = 'S'

                elif ( l_r ==1 and f_b ==-1 ):
                    if(left_right ==1 and front_behind ==1):
                        DIRECTION_FACING = 'E'
                    elif(left_right ==1 and front_behind ==-1):
                        DIRECTION_FACING = DIRECTION_FACING
                    elif(left_right ==-1 and front_behind ==1):
                        DIRECTION_FACING = 'S'
                    elif(left_right ==-1 and front_behind ==-1):
                        DIRECTION_FACING = 'W'
                elif ( l_r ==-1 and f_b ==1 ):
                    if(left_right ==1 and front_behind ==1):
                        DIRECTION_FACING = 'W'
                    elif(left_right ==1 and front_behind ==-1):
                        DIRECTION_FACING = 'S'
                    elif(left_right ==-1 and front_behind ==1):
                        DIRECTION_FACING = DIRECTION_FACING
                    elif(left_right ==-1 and front_behind ==-1):
                        DIRECTION_FACING = 'E'
                elif ( l_r ==-1 and f_b ==-1 ):
                    if(left_right ==1 and front_behind ==1):
                        DIRECTION_FACING = 'S'
                    elif(left_right ==1 and front_behind ==-1):
                        DIRECTION_FACING = 'E'
                    elif(left_right ==-1 and front_behind ==1):
                        DIRECTION_FACING = 'W'
                    elif(left_right ==-1 and front_behind ==-1):
                        DIRECTION_FACING = DIRECTION_FACING
                return OLD_DIRECTION, DIRECTION_FACING
                #now come
    


 


def calculateCamPosition(objects_detected,num_seconds,posFromVO):
    global VO_X
    global VO_Z
    
    new_objects_detected={}
    objects_mid={}
    pos=([0,0],0)
    badPos = ([0,0],0)
    print("Looking...") #trying to triangulate
    new_objects_detected,_,objects_mid,tri = tryTriangulate(num_seconds) #look at camera whilst at new angle
    print("New objects Found:",len(new_objects_detected))
    if(not tri): #if can't triangulate after 5 seconds
        return (False, new_objects_detected,objects_mid,badPos)
   

    #find direction 
   
    #get the corresponding points
    matches=0
    corressponding=[None] *3 #gives corresponding index of new_objects in already objects detected
    for i in new_objects_detected.keys():
        if(i in objects_detected.keys()):
            corressponding[matches]= i
            matches= matches +1

    if(matches<2): #use VO
        print("Could not find at least 2 objects that positions have already been found")
        return (False,new_objects_detected,objects_mid,pos)

    old,new=findCamDirection(new_objects_detected,corressponding)
    print("Camera Direction: from {0} to {1}".format(old,new))

    pos = calculatePossiblePosition(objects_detected,new_objects_detected,corressponding)
    # if(matches==2):#can calculate 2 possible positions for camera
    #     print("Can find possible positions")
       

    #     pos_1,pos_2= calculatePossiblePosition(objects_detected,new_objects_detected,corressponding)
    #     if(pos_1 == [None,None] or pos_2 == [None,None]):
    #         return  (False, new_objects_detected,objects_mid,badPos)
       
    #     print("Using VO position (of {0}) to find correct pos".format(posFromVO))
    #     pos= getAccuratePos(pos_1,pos_2,posFromVO) #now using the feature matcher get the correct position out of the two
    #     if(pos==(None,None)):
    #         return (False, new_objects_detected,objects_mid,badPos)
    #     pos = (pos,distBetweenVectors(pos,[0,0]))
    # else: #more than three corresponding points can find absolute position
    #     print("Can find absolute position")
    #     pos= calculateAbsolutePosition(objects_detected,new_objects_detected,corressponding)
       
    #found a position at this point
   
    
    return  (True,new_objects_detected,objects_mid, pos )     

cam_matrix,distCO= cf.loadCameraCalibration("depth.cal")
cam_matrix=np.array(cam_matrix)
distCO=np.array(distCO)

def getNextFrame():
    global image
    global depth_image
    _,image= imageHub.recv_image() # get image from client
    imageHub.send_reply(b'OK')
    #_,_,image= cf.undistortImage(cam_matrix,distCO,image)
    #image=cv2.resize(image,(image_width,image_height))
    image= imutils.resize(image,width=image_width,height=image_height)
    _,depth_image = imageHub.recv_image()
    imageHub.send_reply(b'OK')
    depth_image= imutils.resize(depth_image,width=image_width,height=image_height)



def calculateDirectionChanges(kp_1,kp_2,matchesMask,num_points,depth_image,prev_depth_image):   
    #see which kp_1 are in kp and which kp_2 are in prev_kp
    #just take 5 points
    x_change = []
    z_change = []
    count=0

    for i in range(len(kp_1)): 
        if(matchesMask[i]):
          
            x_1=int(round(kp_1[i][0][0]))
            y_1=int(round(kp_1[i][0][1]))
                
            x_2=int(round(kp_2[i][0][0]))
            y_2=int(round(kp_2[i][0][1]))
                    
        

            z_1= int(round(depth_image[y_1][x_1] *100 *depth_scale))
            
            z_2= int(round(prev_depth_image[y_2][x_2] *100 * depth_scale))
            
            if(int(z_1) == 0 or int(z_2)==0):
                continue

            x_coord_1,_= calculateOriginOffset(x_1,y_1,z_1)
            x_coord_2,_= calculateOriginOffset(x_2,y_2,z_2)
            x_coord_1 = int(round(x_coord_1))
            x_coord_2 = int(round(x_coord_2))
           
            x_change.append(x_coord_2-x_coord_1)
            z_change.append(z_2-z_1)
            if(count== num_points):
                break
            count= count +1
    if(len(x_change)==0):
        return (None,None)
    return int(round(np.average(x_change))), int(round(np.average(z_change)))


def updateVO():

    global image
    global prev_image
    global depth_image
    global prev_depth_image
    global VO_X
    global VO_Z
    kp_1, kp_2, matchesMask = vo.detectAndMatch(image,prev_image)
    x_change,z_change=calculateDirectionChanges(kp_1,kp_2,matchesMask,KP_NUM,depth_image,prev_depth_image)
    if(x_change is not None):
        VO_X = VO_X + x_change
        VO_Z = VO_Z + z_change
    else:
        print("Not enough key points found to update VO position therefore current Position is unknown, try triangulation to find position")
        VO_X = None
        VO_Z = None
    VO_X= 0
    VO_Z= 0
    prev_depth_image = depth_image.copy()
    prev_image= image.copy()
    return (VO_X,VO_Z)


#other thread will run this function for ever

def VOCalculate():
    global VO_X
    global VO_Z
    global image
    global prev_image
    global depth_image
    global prev_depth_image
    global doVO
    global canVO
    global showBlurs
    prev_image= None
    prev_depth_image = None
    KP_NUM = 8 # number of key points to look for direction change
    while(1):
          
        #get frame data for NS seconds
        #print("VOCalulate")
        if(keyboard.is_pressed('v')):
            doVO=True
        if(doVO): #wait for v to be
            if(VO_X is None):
                print("VO cannot be done as position of robot is currently unknown!")
                doVO=False
                continue
                
                      
            else:
                print("Starting VO")
                posFromVO = updateVO()
                VO_X = posFromVO[0]
                VO_Z = posFromVO[1]
                print("Finished VO: ",VO_X,VO_Z)
              
            doVO=False
            #vo =image # image is constantly being update by main thread so just use this
        





def updateObjectDetected(POS,new_objects_detected,object_mids): #update graph
    #if new id found at to new
    if(POS[0] is None):
        print("Position of Camera is unknown therefore can't update Position of objects!")
        return False
    for i in new_objects_detected.keys():
        


        x=  new_objects_detected[i][0][0][0]
        z=  new_objects_detected[i][0][0][1]
        if(DIRECTION_FACING=='E'):
            x,z= z,x 
        elif(DIRECTION_FACING=='W'):
            x,z= z,x 
    


        start= new_objects_detected[i][1][0] #start how ever so many cm off the camera's centre
        width= new_objects_detected[i][1][1]
        #now translate coords into absolute coords
     

        print("POS:" ,i,x,z)
        print("Start: ",start)


        if(DIRECTION_FACING=='S'):
            new_x = POS[0] + x
            new_z = POS[1] - z
            start = POS[0] + x
        elif(DIRECTION_FACING=='E'):
            new_x = POS[0] + x
            new_z = POS[1] + z
            start = POS[1] + start
        elif(DIRECTION_FACING=='W'):
            new_x = POS[0] - x
            new_z = POS[1] + z
            start = POS[1] + start
    
        elif(DIRECTION_FACING=='N'):
            new_x = POS[0] + x
            new_z = POS[1] + z
            start = POS[0] + start

        #start = POS[0]+start #update where to start now
        dist= int(round(np.sqrt(new_x**2+new_z**2)))                                                                                                                                                                                         
        new_pos=([new_x,new_z],dist)
        #x = ((depth_2_xy) * ( IMAGE_CENTRE[0] - x_p))/fx 
        #
        new_mid_x= int(round(IMAGE_CENTRE[0]- ((new_x*fx)/dist)))  #convert from cm to pixles
        new_mid_y= int(round(IMAGE_CENTRE[1]- ((new_z*fy)/dist)))
        

        object_mids[i]=[new_mid_x,new_mid_y]
        new_objects_detected[i] = list(new_objects_detected[i]) #tupples are immutable
        new_objects_detected[i][0]= new_pos
        new_objects_detected[i][1]= (start,width)
        
        if( i not in  object_ids ): # new object
            print("New object (of id {0}) found, absolute coords are: {1} ".format(i,new_pos))

            objects_detected[i]=(new_pos,(start,width))
            object_ids.append(i)
        print("POS:" ,i,new_pos)
        print("Start: ",start)

    
    graphObjects(new_objects_detected,object_mids)
    updateDirectionsAndRelations(objects_detected)
    return True


getNextFrame() #get first frame

t = threading.Thread(target = VOCalculate)
t.daemon=True #die when main thread dies
t.start() # start thread

prev_depth_image = depth_image.copy()
prev_image= image.copy()

objects_detected,object_ids,obs_mid , _= tryTriangulate(NS)



print("Objects Found:",len(objects_detected))
for i in objects_detected.keys():
    print(i,objects_detected[i])

def updateDirectionsAndRelations(objects_detected):
    global mapped
    global relations
    
    relations = {}
    for i in objects_detected.keys():
        if(i in mapped.keys()): 
            if(not DIRECTION_FACING in mapped[i]):
                mapped[i].append(DIRECTION_FACING)
        else:
            mapped[i]=[DIRECTION_FACING] #its going to mapped north

    
    #now update relations

    for i in objects_detected.keys():  #{i: (([x,z],dist),(start,width)}  (pos,(start,width))
        x_i=  objects_detected[i][0][0][0]
        z_i=  objects_detected[i][0][0][1]
        start_i= objects_detected[i][1][0]
        
        for j in objects_detected.keys(): 
            if(j==i):
                continue
            
            x_j=  objects_detected[j][0][0][0]
            z_j=  objects_detected[j][0][0][1]
            start_j= objects_detected[j][1][0]
            
            left_right=1
            front_behind = 1
            if ( start_i < start_j ): 
                left_right = -1 # i is to the left of j
            if ( z_i <z_j):  # i is behind j
                front_behind=-1

            if(i in relations.keys()):
                relations[i].append({j:[left_right,front_behind]})
            else:
                
                relations[i]= [{j:[left_right,front_behind]}]
        
    
    # NOT FACING NORTH
                



#find direction which camera is facing at start camera is facing North
mapped =  {} #which direction each object has been map



graphObjects(objects_detected,obs_mid) #graph current objects
updateDirectionsAndRelations(objects_detected)


#start VO thread




print("Now starting...")
while True: #continously get frames and update the camera position
    
    getNextFrame()
         
    cv2.imshow('Webcam',image)
    key_pressed= cv2.waitKey(frame_rate)
    if(key_pressed==ord('t')): #try triangulation
        print("VO: ({0},{1})".format(VO_X,VO_Z))
        print("Camera Direction: ",DIRECTION_FACING)
        print("Attemting to find")
        posFromVO= (VO_X,VO_Z)
        triangulate, new_objects,objects_mid,CAM_POS= calculateCamPosition(objects_detected,NS,posFromVO) # num of seconds to try find the camera position
        if(triangulate):
        
            posFromVO= updateVO()

            print("Calculated position from triangulation",CAM_POS)
            print("Updating new objects found(if any) using triangulation measurements")
            #cam pos will be valid here
            updateObjectDetected(CAM_POS[0],new_objects,objects_mid) # if new objects (landmarks) were found in the new frame give them positions
            #update the VO pos
            VO_X = CAM_POS[0][0]
            VO_Z = CAM_POS[0][1]
            X=  CAM_POS[0][0]
            Y=  CAM_POS[0][1]
            posFromVO= (VO_X,VO_Z)
            print("Updating VO to: ",posFromVO)
        else: #can triangulate so use reading from VO  
            doVO=True #other thread will pick this up and do VO
            while(doVO): #wait until VO is done
                getNextFrame()
           
            print("Calculate position from VO:", (VO_X,VO_Z))
            print("Updating new objects found(if any) using VO measurements")
            posFromVO= (VO_X,VO_Z)
            if(VO_X is None): #not valid vo POS
                print("VO failed, try move robot and use triangulation again to find pos")
            else:
                updateObjectDetected(posFromVO,new_objects,objects_mid)
    if(key_pressed==ord('q')): 
        break


cv2.destroyAllWindows()
server_socket.close()


