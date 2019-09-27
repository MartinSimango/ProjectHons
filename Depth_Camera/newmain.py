
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
rate          = jsonData.get("rate") 
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
    x = (depth_2_xy * ( IMAGE_CENTRE[0] - x_p))/fx 
    y = (depth_2_xy * ( IMAGE_CENTRE[1] - y_p))/fy 

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

def getPoints(xs,ys,NUM_POINTS):
    retPoints=[]

    retPoints.append([int((xs[0] +xs[2])/2) , int((ys[0]+ys[2])/2)])
    #retPoints.append([int((xs[0] +xs[1])/3) , int((ys[0]+ys[2])/3)])

    
   
   # for i in range(4):
        #x_r=random.randrange(xs[0],xs[1])
        #y_r=random.randrange(ys[0],ys[2])
    #    retPoints.append([int(xs[0]),int(ys[0])])
        #retPoints.append([x_r,y_r])
    return retPoints
def getDepthInfo(corners): #retrieves depth info for marker
    
    #NUM_POINTS=5;
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
    points= getPoints(xs,ys,1)
    for i in range(len(points)):
        x= points[i][0]
        y= points[i][1]

        x_send.append(x)
        y_send.append(y)
    
    jsonData = json.dumps({"x":x_send,"y":y_send})  
    sock.send(jsonData.encode())
          
    #read depths_back
    data=sock.recv(buffer_size)
    jsonData= json.loads(data.decode())
            
    depth = jsonData.get("depth")
   
    dist  = []
    for i in range(len(depth)):
        if(int(depth[i]) != 0):
            dist.append(int(depth[i])*depth_scale*100)
           
    

    pos=(0,0,0)
    if(len(dist)!=0):
        dist = np.average(dist)
        x_coord, y_coord= calculateOriginOffset(points[i][0],points[i][1],dist)
        pos=(x_coord,y_coord,dist)
        print("POS: ",pos)

    return pos
    
    



def detectMarkerInfo(image):

    markerCorners,markerID, rejectedCandidates = aruco.detectMarkers(image,markerDictionary)
    r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquareDimension,cameraMatrix,distCoeffs)
    markerInfo=[] # number of identified markers
    if(markerID is not None):
        for i in range(len(markerID)):
            #draw the square around makers
            aruco.drawAxis(image,cameraMatrix,distCoeffs,r_vect[i],t_vect[i],0.1)
            aruco.drawDetectedMarkers(image,markerCorners)
            #print(len(markerCorners))
            correctedCorners=getCorners(markerCorners[i][0]) #correct order of corners
            markerInfo.append((markerID[i][0],getDepthInfo(correctedCorners))) #add coords to specific object identified by markerID
    canTrianguatle = (len(markerID) >= 2)
    return markerInfo, canTrianguatle      
            

def calculateCamPosition(objects_detected,image):
    tri= False
    new_objects_detected=[]
    while(not tri):
        new_objects_detected , tri = detectMarkerInfo(image) #look at camera whilst at new angle
    #do some calculations with new_objects and old objects
    id_1,pos_1= new_objects_detected[0][0], new_objects_detected[0][1] 
    id_2,pos_2= new_objects_detected[1][0], new_objects_detected[1][1]
    #match this id's with previous seen objects
    x_s=[]
    y_s=[]
    z_s=[]
    #get the corresponding points
    corressponding=[]
    sorted_objects_z = [] #for sorting closets objects from camera points if facing z direction
    sorted_objects_x = [] #for sorting closets objects from camera points if facing x direction
    for i in range(len(objects_detected)):
        for j in range(len(new_objects_detected)):
            if(objects_detected[j][0]==new_objects_detected[i][0]): #same object 
                corressponding.append(i,j) # object i in new objects is the same object is object j
                sorted_objects_z.append(new)
                sorted_objects_x.append()
                break #found corresponding object now find next one
    
    #assuming facing 1 direction (z direction)
    #sort objects by distance
    sorted_objects =  

    #cross check not valid check facing other direction(x direction)
            
    
objects_detected=[]
triangulate=False
while True:
     
    rpiName,image= imageHub.recv_image() # get image from client
     
    imageHub.send_reply(b'OK')
    image= imutils.resize(image,width=image_width,height=image_height)
    
    
    if(not triangulate): #detect objects if you cant trianglute
        objects_detected,triangulate=detectMarkerInfo(image)
    if(triangulate): #enough points known to triangluate camera position
        print("Can triangulate")
    
    

    cv2.imshow('Webcam',image)
    if(cv2.waitKey(30)==ord('t')):
        calculateCamPosition(objects_detected,image)

cv2.destroyAllWindows()
server_socket.close()



    # if(count == rate): #past the first frame
    #     total_distance=0 
    #     distances_pos=[]
    #     distances_neg=[]
    #     keyPoint_indices=[]
    #     kp_1,kp_2,matchesMask,H=detectAndMatch(image,prev_image)
    #     if(matchesMask is not None ):
    #         x1_s=[]
    #         x2_s=[]
    #         y1_s=[]
    #         y2_s=[]
    #         for i in range(len(matchesMask)):
    #             if(matchesMask[i]): #if a good match after and in RANSAC model

    #                 x_1=int(round(kp_1[i][0][0]))
    #                 y_1=int(round(kp_1[i][0][1]))
                
    #                 x_2=int(round(kp_2[i][0][0]))
    #                 y_2=int(round(kp_2[i][0][1]))
                    
    #                 x1_s.append(x_1)
    #                 x2_s.append(x_2)
    #                 y1_s.append(y_1)
    #                 y2_s.append(y_2)
    #                 keyPoint_indices.append(i)
    #             if(len(x1_s)==MAX_DEPTH_POINTS): #only get up to 20 matching points max
    #                 break
    #                 #send coords off to get back depth
    #         jsonData = json.dumps({"x1":x1_s,"y1":y1_s,"x2":x2_s,"y2":y2_s})       
    #         sock.send(jsonData.encode())
          
    #         #read depths_back
    #         data=sock.recv(buffer_size)
    #         jsonData= json.loads(data.decode())
            
    #         depth_1 = jsonData.get("depth_1")
    #         depth_2 = jsonData.get("depth_2")
            
    #         x_change, y_change = calculateDirectionChanges(kp_1,kp_2,keyPoint_indices,depth_2) 
            
    #         depth_len = len(depth_1)

    #         for i in range (depth_len):
    #             if(int(depth_1[i])==0 or int(depth_2[i])==0): #no depth for that point
    #                 continue
    #             dist=(int(depth_2[i])-int(depth_1[i]))*depth_scale*100  #get to cm
    #             if(dist>=0):
    #                 distances_pos.append(dist)
    #             else:
    #                 distances_neg.append(dist)
    #             #print("Key Points: ",kp_1[i],kp_2[i])  #check to see how similar key points are
    #             #print("Depth_1: ", int(depth_1[i])*depth_scale*100)
    #             #print("Depth_2: ", int(depth_2[i])*depth_scale*100)
              
               
           
    #         if(distances_pos): #some depth measurement in z directions
    #             total_distance=np.min(distances_pos)
    #             if(distances_neg):
    #                 if(abs(np.max(distances_neg))<total_distance):
    #                     total_distance=np.max(distances_neg)
    #         #now measure distance change in x and y direction
    #         #x_change,y_change=calculateDirectionChanges(kp_1,kp_2,matchesMask)
                   
              
              
    #     #match points -
    #     #use ransac to get best inliers -
    #     # from homography matrix get rotation matrix and translation vectors (using decomposHomography)
    #     #compute points cloud for key points 
    #     #use points clouds to get rotation and translation matrices
