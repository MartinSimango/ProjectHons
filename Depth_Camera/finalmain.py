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
from mpl_toolkits import mplot3d

import threading
import trilaterate as tl
#lookup table which stores positions of landmarks

#two threads -one thread gets image, second thread gets depth_image
BUFFER_SIZE          = 4096 #max size of data sent
SERVER_PORT_NUM      = int(sys.argv[1])
IMAGE_HUB_PORT_NUM   = SERVER_PORT_NUM + 1
DEPTH_IMAGE_PORT_NUM = SERVER_PORT_NUM + 2 

ROBOT_POSE = (0,0,0,0) # x,y,z, orientation 
MAP_RANGE = 2 # how many meters should the depth map see up to
#lock for sychronizing image and depth_image calculations
IMAGE_LOCK = threading.Lock()
ROBOT_POSE_KNOWN = False

TRI_TIMES=0
#FOR PLOOTING
plt.ion() #allow plt to be continously updated
FIG=plt.figure()
def init_socket(port_num):
    #CREATE TCP SOCKET
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind socket
    server_socket.bind(('',port_num))
 
    #start listening
    server_socket.listen(5) 
    return server_socket



def getDepthCamInfo(server_socket): #

    sock,address = server_socket.accept() #wait for connection from client
    print(f"Connection from {address} received!")
    
    #get camera_information
    #would normally use a header for protocol but because we know what the client is sending here the is no use
    data=sock.recv(BUFFER_SIZE)
    jsonData= json.loads(data.decode())
    return jsonData


def getNextFrame():
   
    _,image= IMAGE_HUB.recv_image() # get image from client
    IMAGE_HUB.send_reply(b'OK')
    image= imutils.resize(image,width=IMAGE_WIDTH,height=IMAGE_HEIGHT)
    
    _,depth_image= IMAGE_HUB.recv_image() # get image from client
    IMAGE_HUB.send_reply(b'OK')

    depth_image= imutils.resize(depth_image,width=IMAGE_WIDTH,height=IMAGE_HEIGHT)
    return image,depth_image




def calculateRobotPos(image,depth_image):
    #todo 
    #FIND MARKERS
    
    num_seconds= 1 #give 3 seconds to find markers
    start_time= time.time()
    landmarksFound={}
    while(True):
        landmarksFound = tl.findMarkers(image)
       
        if(time.time()-start_time >num_seconds or len(landmarksFound)>2): #time is up stop trying to triangulate and switch to something eles 
            break
        getNextFrame()
    if(len(landmarksFound)<3):
        print("Could not find 2 or more landmarks. Please move robot")
        return False, None
    #found 2 or more landmarks now find distances to them
    landmarkDistances,focals= tl.getLandmarkDistances(landmarksFound,depth_image,DEPTH_SCALE)
   
    if(len(landmarkDistances)<3):
        print("Could not find distances to enough landmarks. Please move robot!")
        return False, None
    focal_average=[]
    for id_s in landmarkDistances.keys():
        d,mid_x,mid_y = landmarkDistances[id_s] #convert the pixels into cm's
       
        focal_average.append(focals[id_s])
        x_offset,y_offset= calculateOriginOffset(mid_x,mid_y,d)#focals[id_s],focals[id_s])
        landmarkDistances[id_s] = (d,x_offset,y_offset) 
        print("Landmark distances:",id_s,landmarkDistances[id_s],np.sqrt(d**2-x_offset**2))
    #look in lookup tables where markers are
    #use depth_image to find distance these marks
    #use triliteration to find pose of robot
    fur_x,fur_y,fur_dist= getFurthest(5,int(CX),int(CY),depth_image)
    print("Furthest: ",fur_x,fur_y,fur_dist)
    new_pose=tl.trilateratePos(landmarkDistances,fur_dist)
    if(new_pose is None): #could not find the robot's pose
        return False,None

    return True,new_pose
def calculateOriginOffset(x_p,y_p,depth_2_xy,fx=None,fy=None):
    if(fx is None):
        fx=FX
    if(fy is None):
        fy=FY
    x = ((depth_2_xy) * ( (x_p - CX) ))/fx
    y = ((depth_2_xy) * ( - (y_p-CY) ))/fy

    return (x,y)

#get furthest point seen in the middle of the camera
def getFurthest(width,image_center_x,image_center_y,depth_image):
    x=None 
    y=None
    max_dist=0
    return 0,0,200
    for i in range(width):
        for j in range(width):
            dist_1 = depth_image[image_center_y+i][image_center_x+j] * DEPTH_SCALE *100 # get to cm's
            dist_2 = depth_image[image_center_y-i][image_center_x+j] * DEPTH_SCALE *100 # get to cm's
            dist_3 = depth_image[image_center_y+i][image_center_x-j] * DEPTH_SCALE *100 # get to cm's
            dist_4 = depth_image[image_center_y+i][image_center_x-j] * DEPTH_SCALE *100 # get to cm's
            dist = max(dist_1,dist_2,dist_3,dist_4)
            if(dist>max_dist):
                max_dist=dist
           
    # for i in range(len(depth_image)):#look through all rows
    #     for j in range(len(depth_image[0])): #look through columns image_center-width to image_center+1
    #         if(j>= image_center_x-width and  j <= image_center_x+width ):
    #             dist = depth_image[i][j] * DEPTH_SCALE *100 # get to cm's
    #             if(dist>max_dist):
    #                 max_dist=dist
    #         else:
    #             continue
   
    return x,y, 200
def filterDepthImage(depth_image,map_range):
  
    points = {}
    for i in range(len(depth_image)):
        for j in range(len(depth_image[0])):
            depth_cm =int(depth_image[i][j]) * DEPTH_SCALE *100
            y_height = (depth_cm * ( IMAGE_CENTRE[1] - i ) )/FY  # pixel height from the ground #236.63084547568047
            if((depth_cm> map_range*100) or y_height<-14):   #filter out ground and anything futher then dto +2cm
                depth_cm=0
            if(depth_cm==0):
                continue

            x,y= calculateOriginOffset(j,i,depth_cm)
           
            points[x,y]=depth_cm
            #points[int(x),int(y)]=depth_cm
    return points


x_points=[]
y_points=[]
z_points=[]
def mapEnvironment(robot_pose,points,add_to_angle):
    global TRI_TIMES
    ax = plt.axes(projection="3d")
   
    r_x = robot_pose[0]
    r_y = robot_pose[1]
    r_z = robot_pose[2] 
    angle= -(robot_pose[3])
   
    basis= np.matrix([[1, 0], [0, 1]])
    rotMatrix = np.matrix([[np.cos(angle), -np.sin(angle)], 
                            [np.sin(angle),  np.cos(angle)]])
    new_basis= rotMatrix*basis
    
  
    rob_pos=np.array([r_x,r_y])
    rob_pos.shape=(2,1)
  
    count=0
 
    # if(TRI_TIMES==0):
    for x,y in points.keys():
        count = count +1
        if(count%50!=0):
            #count=0
            continue
        #calculate new point position in terms of new_basis
        XY= np.array([x,points[x,y]])
        XY.shape=(2,1)
        rotated_XY= rotMatrix*XY
        rotated_trans_X = rotated_XY[0,0] + r_x
        rotated_trans_Y = rotated_XY[1,0] + r_y

    
        x_points.append(rotated_trans_X)
        y_points.append(rotated_trans_Y)
        z_points.append(y)

       

    print("LEN: ",len(x_points))
    area = 1#np.pi*5
    
    ax.scatter3D(x_points,y_points,z_points,s=area,color='black')
  #  ax.scatter3D(x_points,y_points,z_points,s=area,c=y_points,cmap='hot',alpha=0.5)
    #ax.scatter3D(new_x_points,new_y_points,new_z_points,s=area,color='purple')
    ax.scatter3D(r_x,r_y,0,s=(np.pi*10),c='green')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    
    plt.pause(0.001) # plot non blocking
    
#start the server socket

print("Server started!")
SERVER_SOCKET= init_socket(SERVER_PORT_NUM) #start socket for listening to input for camera information
jsonData= getDepthCamInfo(SERVER_SOCKET)


DEPTH_SCALE   = jsonData.get("depth_scale") 
IMAGE_WIDTH   = jsonData.get("width") 
IMAGE_HEIGHT  = jsonData.get("height") 
FRAME_RATE    = jsonData.get("frame_rate") 
RATE          = jsonData.get("rate")  # controls how fast should the server request client for depth information for key points
FX            = jsonData.get("fx") #602.881168321761  #value from depth.cal # 
FY            = jsonData.get("fy") #603.1732052013384 #value from depth.cal 
CX            = jsonData.get("cx")
CY            = jsonData.get("cy")
DIST_COEFFS    = jsonData.get("distCoeffs")

IMAGE_CENTRE =((IMAGE_WIDTH)/2.0,IMAGE_HEIGHT/2.0)
print("Received: ", jsonData) #get data from camera

#start image servers

port= "tcp://*:"+str(IMAGE_HUB_PORT_NUM)
IMAGE_HUB = imagezmq.ImageHub(open_port=port)

#port= "tcp://*:"+str(depth_image_port_num)
#depthImageHub = imagezmq.ImageHub(open_port=port)


def getFromUser():
    return float(input("What angle to add?"))

IMAGE= None
DEPTH_IMAGE=None
def ImageThread():
    global IMAGE,DEPTH_IMAGE
    global ROBOT_POSE
    global ROBOT_POSE_KNOWN
    global TRI_TIMES
    while True:
        #if(ROBOT_POSE_KNOWN):
        #    IMAGE_LOCK.acquire()
    
      
        IMAGE,DEPTH_IMAGE=getNextFrame()
        
        #try calcluate robot position
        #wait for robot to move
        cv2.imshow('Image',IMAGE)
        cv2.imshow('Depth_Image',DEPTH_IMAGE)
        
        key_pressed= cv2.waitKey(FRAME_RATE)
        if(key_pressed==ord('q')): 
            break
        elif(key_pressed==ord('t')):
            
            ROBOT_POSE_KNOWN,ROBOT_POSE=calculateRobotPos(IMAGE,DEPTH_IMAGE) 
            
            #ROBOT_POSE_KNOWN,ROBOT_POSE=True,(0,0,0,0)
            if(ROBOT_POSE_KNOWN):
                add_to_angle=0
                # if(TRI_TIMES == 1):
                #     #plt.ioff() #allow plt to NOT be continously updated  
                #     add_to_angle= getFromUser()
                #     print("Gonna add ",add_to_angle)

                print("Found robot pose, robot pose: ",ROBOT_POSE)
                print("Now time to map!")
                print("Starting to map!")
                points= filterDepthImage(DEPTH_IMAGE,MAP_RANGE)
                print("OK")
                mapEnvironment(ROBOT_POSE,points,add_to_angle)
                print("Finished mapping")
                ROBOT_POSE_KNOWN=False 
                TRI_TIMES = 1    
             
            else:
                print("Could not find robots pose!\nPlease move the robot!") 
                #don't release lock
        
    print("FINISHED!")
#thread for moving robot?

ImageThread()


