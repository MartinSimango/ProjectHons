
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
    #fov=(float(x_p)/image_width) *np.deg2rad(69.4)
    x = ((depth_2_xy) * ( IMAGE_CENTRE[0] - x_p))/fx 
    #x = depth_2_xy * np.sin(fov)
    y = ((depth_2_xy) * ( IMAGE_CENTRE[1] - y_p ))/fy 

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
    
    NUM_POINTS=20;
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
           
    

    pos=([0,0,0],0)
    if(len(dist)!=0):
        dist= np.min(dist)
        mid_x= int((xs[0] +xs[1])/2) 
        mid_y= int((ys[0]+ys[2])/2)
        x_coord, y_coord= calculateOriginOffset(mid_x,mid_y,dist)
        dist_orth = np.sqrt(np.square(dist)-np.square(x_coord))# essentially  dist_orth = sqrt(z^2-x^2)
        pos=([round(x_coord),round(y_coord),round(dist_orth)],dist) 
        print("POS: ",id_o,pos)

    return pos
    
    



def detectMarkerInfo(image):

    markerCorners,markerID, rejectedCandidates = aruco.detectMarkers(image,markerDictionary)
    r_vect,t_vect, _ =aruco.estimatePoseSingleMarkers(markerCorners,arucoSquareDimension,cameraMatrix,distCoeffs)
    markerInfo=[] # number of identified markers
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
          
    canTrianguatle = (len(markerInfo) >= 2) 
    return markerInfo, canTrianguatle      
            
def distBetweenVectors(A,B):
    #B is assummed to be further
    
    x_change= np.square(B[0]-A[0])
    zorth_change= np.square(B[3] -A[3])
    return np.sqrt(x_change+zorth_change)


def calculateCamPosition(objects_detected):
    tri= False
    new_objects_detected=[]
    while(not tri):
        _,image= imageHub.recv_image() # get image from client
        imageHub.send_reply(b'OK')
        image= imutils.resize(image,width=image_width,height=image_height)
        print("Looking...")
        new_objects_detected , tri = detectMarkerInfo(image) #look at camera whilst at new angle
    #do some calculations with new_objects and old objects
        id_1=objects_detected[0][0]
        id_2=objects_detected[1][0]
    #get the corresponding points
        corressponding=[None] *2 #gives corresponding index of new_objects in already objects detected
        for j in range(len(new_objects_detected)):
            if(id_1==new_objects_detected[j][0]): #same object 
                corressponding[0]=j # object i in new objects is the same object is object j
            if(id_2==new_objects_detected[j][0]): #found corresponding object now find next one
                corressponding[1]=j
      
    #create triangle where C is camera and A and B are other positions
    print("Can triangulate!!!")
    new_A_vec = new_objects_detected[corressponding[0]][1][0] # position of A with camera at origin
    new_B_vec = new_objects_detected[corressponding[1]][1][0]
    
    A= objects_detected[0][1][0]
    B= objects_detected[1][1][0]
    x1,y1,z1 = A 
    
    A=[x1,z1]
    x2,y2,z2 = B
    
    B=[x2,z2]
    A_mag=  np.linalg.norm(A)
    B_mag=  np.linalg.norm(B)
    A_dot_B= np.dot(A,B)

    current_A_y=  new_objects_detected[corressponding[0]][1][0][1]

    new_C_vec = [0,0]
    new_A_vec =[new_A_vec[0],new_A_vec[2]]
    new_B_vec =[new_B_vec[0],new_B_vec[2]]
    print("A",new_A_vec)
    print("B",new_B_vec)
    
    AB= np.subtract(new_A_vec,new_B_vec)
    AC= np.subtract(new_A_vec , new_C_vec)
    BA = np.subtract(new_B_vec , new_A_vec)
    BC = np.subtract(new_B_vec , new_C_vec)
    #find angle between AC and AB (using dot product) AC dot AB =|AC||AB|cosa where
    #a is angle between AC and AB
    #b is angle between BC and BA
    AC_mag= np.linalg.norm(AC) 
    AB_mag=np.linalg.norm(AB)
    BC_mag= np.linalg.norm(BC)
    BA_mag= np.linalg.norm(BA)
    # print("AB:",AB)
    # print("AC:",AC)
    # print("BC:",BC)
    # print("BA:",BA)
    a = np.arccos(np.dot(AB,AC)/(AB_mag*AC_mag))
    ao = np.arccos(np.dot(A,AC)/(A_mag*AC_mag))
    #a= np.rad2deg(a)
    #b= np.rad2deg(b)
    #a = np.deg2rad(round(a))
    #b=  np.deg2rad(round(b))
    
    
    

    #have angles between vectors now wer need to find C true position
    # in relation to the origin need to find x1 y1 and z3 of C in relation to origin
    
    #we know AB dot AC = |AB||AC|cosa 
    # and we know BA dot BC = |BC||BA|cosb
    # and we know |AB cross AC| dot BC = 0 as we know this vectors form a triangle and hence will lie on 1 plane
    
    
    # we have 3 unknows x3,y3 and z3 using some linearly algebra solving these
    # we can find them  
    # which gives us
    #{ (x1-x2).(x2-x3) + (y1-y2).(y2-y3) + (z1-z2).(z2-z3) = |AC||AB|cosa }
    #form matrix to solve position of camera relative to origin
    #Mp = S

    #get neccesary values to compute position
    
    #place holders to make things easier
  
    M= np.matrix([ [x2-x1,z2-z1],
                 [-x1,-z2]])
    print("a",np.rad2deg(a))
    #print("b",np.rad2deg(ao))
 
    S11=AB_mag*AC_mag*np.cos(a) - np.square(A_mag) + A_dot_B
    S12=A_mag*AC_mag*np.cos(ao) - np.square(A_mag) 
    S= np.array([S11,S12])
    S.shape=(2,1)
    print("M:",M)
    print("S:",S)
    #M_inverse= np.linalg.inv(M)
    #cameraPos= M_inverse * S 
    #print("O:",cameraPos)
   
    m=np.linalg.lstsq(M, S,rcond=None)[0]
    z_dist=np.sqrt(np.square(m[0,0]) +np.square(m[1,0]))
    
    y_change= y1 - current_A_y 
    new_pos= ([m[0,0],y_change,m[1,0]],z_dist)
    print("Cam Pos: ",new_pos)
    return new_pos

            
    
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
        print("Attemting to find")
        calculateCamPosition(objects_detected)

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
