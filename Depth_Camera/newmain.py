
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
buffer_size=4096 #max size of data sent

(X,Y,Z)= (0.0,0.0,0.0) #starting coords #camera coords

prev_image = None
prev_depth_frame = None
count=0
total_distance = 0
MAX_DEPTH_POINTS=50 # how many depth points we want to ask for
#create image hub and start start
PORT= "tcp://*:"+sys.argv[1]


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
fx            = jsonData.get("fx")
fy            = jsonData.get("fy")
cx            = jsonData.get("cx")
cy            = jsonData.get("cy")
distCoeffs    = jsonData.get("distCoeffs")



print("Received: ", jsonData) #get data from camera

#aruco markers
markerCorners=[]
markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
arucoSquareDimension= 0.048 #length of side of square on aruco marker

cameraMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]) #form the camera matrix
IMAGE_CENTRE =(int(image_width)/2,int(image_height)/2)

#given x and y in pixel coords get x and y coords in cm's
def calculateOriginOffset(x_p,y_p,depth_2_xy):
    #print("I",image_width)
   
    #calculate X and Y change using some trig, 
    # Width  = Z*w/Fx
    # Height = Z*y/Fy
    #fov=(float(x_p)/image_width) *np.deg2rad(69.4)
    x = ((depth_2_xy) * ( cx - x_p))/fx 
    #x = depth_2_xy * np.sin(fov)
    y = ((depth_2_xy) * ( cy- y_p ))/fy 

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
        cv2.circle(image,(x, y), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[0], ys[0]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[1], ys[1]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[2], ys[2]), 5, (0,255,0), -1) #just to visualize point
    #cv2.circle(image,(xs[3], ys[3]), 5, (0,255,0), -1) #just to visualize point
    #retPoints.append([int((xs[0] +xs[1])/3) , int((ys[0]+ys[2])/3)])

    
   
   # 
    #    retPoints.append([int(xs[0]),int(ys[0])])
        #retPoints.append([x_r,y_r])
    return retPoints
def getDepthInfo(corners,id_o,image): #retrieves depth info for marker
    
    NUM_POINTS=50
    #get the y coords
    xs=[]
    ys=[]
    x_send=[]
    y_send=[]
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
    points= getPoints(xs,ys,NUM_POINTS,image)
    for i in range(len(points)):
        x= points[i][0]
        y= points[i][1]

        x_send.append(x)
        y_send.append(y)
    

    jsonData =  json.dumps({"Request": "Tri_Depth"}) #first tell client what you are requesting
    sock.send(jsonData.encode())

    jsonData = json.dumps({"x":x_send,"y":y_send})  
    sock.send(jsonData.encode())
          
    #read depths_back
    data=sock.recv(buffer_size)
    jsonData= json.loads(data.decode())
            
    depth = jsonData.get("depth")
   
    dist  = []
    for i in range(len(depth)):
        if(int(depth[i]) != 0):
            dist.append([int(depth[i])*depth_scale*100])
           
    

    pos=([0,0],0)
    if(len(dist)!=0):
        dist= np.min(dist)
        mid_x= int((xs[0] +xs[1])/2) 
        mid_y= int((ys[0]+ys[2])/2)
        x_coord, y_coord= calculateOriginOffset(mid_x,mid_y,dist)
        dist_orth = np.sqrt(np.square(dist)-np.square(x_coord))# essentially  dist_orth = sqrt(z^2-x^2)
        pos=([int(x_coord),int(dist_orth)],dist) 
        print("POS: ",id_o,pos)

    return pos
    
    



def tryTriangulate(image,num_seconds,max_find=False): #try finding objects for num_seconds seconds #try finding max objects

    canTriangulate= False
    
    #move funcionality to tryTriangulate
    start_time = time.time()
    objects_currently_detected = []
    object_ids_current =[]
    while(not canTriangulate):
    
        markerCorners,markerID, rejectedCandidates = aruco.detectMarkers(image,markerDictionary)
        r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquareDimension,cameraMatrix,distCoeffs)
        markerInfo=[] # number of identified markers
        markerID=[]
        if(markerID is not None):
            for i in range(len(markerID)):
                #draw the square around makers
                #aruco.drawAxis(image,cameraMatrix,distCoeffs,r_vect[i],t_vect[i],0.1)
                aruco.drawDetectedMarkers(image,markerCorners)
                #print(len(markerCorners))
            
                correctedCorners=getCorners(markerCorners[i][0]) #correct order of corners
                pos=getDepthInfo(correctedCorners,markerID[i][0],image)
                if(pos!=([0,0,0],0)):
                    markerInfo.append((markerID[i][0],pos)) #add coords to specific object identified by markerID
                    markerID.append(markerID[i][0])
        
        if(len(markerInfo)>len(objects_currently_detected)):
            objects_currently_detected = markerInfo.copy()
            object_ids_current = markerID.copy()
        canTriangulate = (len(objects_currently_detected) >= 2) 

        if(max_find) : #keep looking
            canTriangulate=False
    
        if(time.time()-start_time >num_seconds): #time is up stop trying to triangulate and switch to something eles 
            if(len(objects_currently_detected)>=2): #incase we have max_find ends up estting can_triangulate right before time is up
                canTriangulate= True   
            break
        
        print("Number Objects found so far: ",len(objects_currently_detected))
        
        if(not canTriangulate): #if you can't triangulate keep looking for more obects
            _,image= imageHub.recv_image() # get next image from client to try triangulate 
            imageHub.send_reply(b'OK')
            image= imutils.resize(image,width=image_width,height=image_height)
            cv2.imshow('Webcam',image)  
            if(cv2.waitKey(30)==ord('s')): #stop looking
                break


    return objects_currently_detected,object_ids_current, canTriangulate      
            
def distBetweenVectors(A,B):
    #B is assummed to be further
    
    x_change= np.square(B[0]-A[0])
    z_change= np.square(B[1]-A[1])
    return np.sqrt(x_change+z_change) #dist between A and B



def calculatePossiblePosition(objects_detected,new_objects_detected,corressponding):
    
    new_A_vec = new_objects_detected[corressponding[0]][1][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[corressponding[1]][1][0]

    #object A and B absolute positions
    A= objects_detected[0][1][0]
    B= objects_detected[1][1][0]
    x1,z1 = A 

   

    x2,z2 = B
    A_mag=  np.linalg.norm(A)
    B_mag=  np.linalg.norm(B)


    new_C_vec = [0,0]

    print("A",new_A_vec)
    print("B",new_B_vec)

   
    AC= np.subtract(new_A_vec , new_C_vec)
   
    BC = np.subtract(new_B_vec , new_C_vec)
    
    AC_mag= np.linalg.norm(AC) 
   
    BC_mag= np.linalg.norm(BC)


    a_= 2*x1
    b_= 2*z1
    C_= np.square(A_mag) - np.square(AC_mag)
    d_= 2*x2 
    e_= 2*z2
    F_ =np.square(B_mag) - np.square(BC_mag)
    M_= e_-b_
    N_= F_-C_
    O_= d_-a_

    P_= np.square(M_) + np.square(O_)
    Q_= -d_*np.square(M_) - 2*N_*O_ + e_*M_*O_
    R_= np.square(N_) - e_*M_*N_ + F_*np.square(M_)
    
    #Px^2 + Qx +R=0
    #apply quadrict formula to solve for possible positions for x
   
    x3_1= (-Q_ + (np.sqrt(np.square(Q_)-4*P_*R_)) ) / (2*P_)
    x3_2= (-Q_ - (np.sqrt(np.square(Q_)-4*P_*R_)) ) / (2*P_)
    z3_1= float('nan')
    z3_2= float('nan')
    if(not math.isnan(x3_1) and not math.isnan(x3_2)):
        x3_1= int(x3_1)
        x3_2= int(x3_2)
        z3_1= int((N_-x3_1*O_) /M_)
        z3_2= int((N_-x3_2*O_) /M_)
    else:
        return ([None,None],[None,None]) # cant get valid position
    pos_1= [x3_1,z3_1]
    pos_2= [x3_2,z3_2]
    print("Pos 1: ",pos_1)
    print("POS 2: ",pos_2)
    #cant use matrix approach because A[0] is not necessarily in the x direction 
    return pos_1,pos_2

def calculateAbsolutePosition(objects_detected,new_objects_detected,corressponding):
    new_A_vec = new_objects_detected[corressponding[0]][1][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[corressponding[1]][1][0]
    new_C_vec = new_objects_detected[corressponding[2]][1][0]

    #object A and B absolute positions
    A= objects_detected[0][1][0]
    B= objects_detected[1][1][0]
    C= objects_detected[2][1][0]
    
    x1,z1 = A 

    x2,z2 = B
  
    x3,z3 = C 
   

  
    A_mag=  np.linalg.norm(A)
    B_mag=  np.linalg.norm(B)
    C_mag=  np.linalg.norm(C)


    new_D_vec = [0,0] # where camera is

    print("A",new_A_vec)
    print("B",new_B_vec)
    print("C",new_C_vec)

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
    pos_x = int(pos_x_z[0,0])
    pos_z = int(pos_x_z[1,0])
    pos=  ([pos_x,pos_z], distBetweenVectors([pos_x,pos_z],[0,0]))
    print("Abs pos ",pos)

    return pos

def getAccuratePos(pos_1,pos_2,posFromVO):
    if(posFromVO==(None,None)):
        return (None,None)
    dist_1 = distBetweenVectors(posFromVO,pos_1)
    dist_2 = distBetweenVectors(posFromVO,pos_2)
    if( dist_1 <dist_2): #find the pos closest to the VO position
        return pos_1
    return pos_2


def calculateCamPosition(objects_detected,num_seconds,posFromVO):
   
    new_objects_detected=[]
    
    pos=([0,0],0)
    badPos = ([0,0],0)
    print("Looking...") #trying to triangulate
    new_objects_detected,_, tri = tryTriangulate(image,num_seconds) #look at camera whilst at new angle
    if(not tri): #if can't triangulate after 5 seconds
        return (False, new_objects_detected,badPos)
   

  
    #get the corresponding points
    matches=0
    corressponding=[None] *3 #gives corresponding index of new_objects in already objects detected
    for i in range(len(objects_detected)):
        for j in range(len(new_objects_detected)):
            if(object_ids[i]==new_objects_detected[j][0]): #same object 
                corressponding[i]=j # object i in new objects is the same object is object j
                matches= matches +1

    if(matches<2):
        print("Could not find at least 2 objects that positions have already been found")
        return (False,new_objects_detected,pos)
    if(matches==2):#can calculate 2 possible positions for camera
        print("Can find possible positions")
        pos_1,pos_2= calculatePossiblePosition(objects_detected,new_objects_detected,corressponding)
        if(pos_1 == [None,None] or pos_2 == [None,None]):
            return  (False, new_objects_detected,badPos)
        pos= getAccuratePos(pos_1,pos_2,posFromVO) #now using the feature matcher get the correct position out of the two
        if(pos==[None,None]):
            return (False, new_objects_detected,badPos)
        pos = (pos,distBetweenVectors(pos,[0,0]))
    else: #more than three corresponding points can find absolute position
        print("Can find absolute position")
        pos= calculateAbsolutePosition(objects_detected,new_objects_detected,corressponding)

    return  (True,new_objects_detected, pos )     


    
frames_recieved = 0
copyPreviousFrame= True
num_seconds=5
CAM_POS=(0,0)


#first get initial objects
rpiName,image= imageHub.recv_image() # get image from client
    
imageHub.send_reply(b'OK')
image= imutils.resize(image,width=image_width,height=image_height)
frames_recieved=  frames_recieved +1
VO_pos= (None,None)
objects_detected,object_ids, _ = tryTriangulate(image,5,True)

def updateObjectDetected(POS,new_objects_detected):
    #if new id found at to new
    for i in range(len(new_objects_detected)):
        id_i=new_objects_detected[i][0]
        if( id_i not in  object_ids ): # new object
            x=  new_objects_detected[i][1][0][0]
            z=  new_objects_detected[i][1][0][1]
            #now translate coords into absolute coords
            new_x= POS[0]+x
            new_z= POS[1]+z
            dist= np.sqrt(new_x**2+new_z**2)
            new_pos=([new_x,new_z],dist)
            #add new object 
            print("New object found absolute coords are: ",new_pos)
            objects_detected.append((id_i,new_pos))
            object_ids.append(id_i)


while True: #continously get frames and update the camera position
     
    rpiName,image= imageHub.recv_image() # get image from client
     
    imageHub.send_reply(b'OK')
    image= imutils.resize(image,width=image_width,height=image_height)
    frames_recieved=  frames_recieved +1
    
    if(frames_recieved > 1): #now we have two consecutive frames (VO)
         kp_1,kp_2,matchesMask,H=vo.detectAndMatch(image,prev_image) # get the matching key points in 
         VO_pos= vo.getPositionFromKeyPoints(sock,kp_1,kp_2,matchesMask,depth_scale,fx) # perform VO
         print("VO: ",VO_pos)
    
              
    cv2.imshow('Webcam',image)

    if(cv2.waitKey(30)==ord('t')):
        print("Attemting to find")
        triangulate, new_objects,CAM_POS= calculateCamPosition(objects_detected,num_seconds,VO_pos) # num of seconds to try find the camera position
        if(triangulate):
            print("Calculated position from triangulation",CAM_POS)
            print("Updating new objects found(if any) using triangulation measurements")
            updateObjectDetected(CAM_POS,new_objects) # if new objects (landmarks) were found in the new frame give them positions
        else: #can triangulate so use reading from VO
            if(frames_recieved > 1):
                print("Calculate position from VO:", (vo.VO_X,vo.VO_Z))
                print("Updating new objects found(if any) using VO measurements")
                updateObjectDetected(VO_pos,new_objects)
            else:
                print("Too early to say anything about cam position")


    if(copyPreviousFrame):    #when to get previous frame ()
        prev_image=image.copy()
    if(cv2.waitKey(30)==ord('q')): 
        break


cv2.destroyAllWindows()
server_socket.close()


