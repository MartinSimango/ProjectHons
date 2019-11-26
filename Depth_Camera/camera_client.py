import pyrealsense2 as rs #for accessing camera
import cv2
import numpy as np
import json #for sending neat forms of data
import sys
import imagezmq
import time

#for threading
import threading
import socket
from queue import Queue

#some global variables

FRAME_RATE=15
RATE=20
HOST_IP= sys.argv[1]
PORT= sys.argv[2]
depth_image=[]
buffer_size=4096
#cofigure camera
pipeline = rs.pipeline()
config= rs.config()
config.enable_stream(rs.stream.depth,640,480,rs.format.z16,FRAME_RATE)
config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,FRAME_RATE)

#Read frames
pipeline.start(config)

#get the instrinsic parameters from the camera
profile=pipeline.get_active_profile()
depth_profile=rs.video_stream_profile(profile.get_stream(rs.stream.color))
depth_intrinsics=depth_profile.get_intrinsics()
w,h= depth_intrinsics.width, depth_intrinsics.height

#get depth scale
depth_sensor= profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

#for aligning streams
align_to = rs.stream.color
align = rs.align(align_to)
#get camera matrix details
fx= depth_intrinsics.fx
fy= depth_intrinsics.fy
cx= depth_intrinsics.ppx
cy= depth_intrinsics.ppy
dist_coeffs= depth_intrinsics.coeffs


SERVER_PORT_NUM      = int(sys.argv[2])
IMAGE_HUB_PORT_NUM   = SERVER_PORT_NUM + 1
DEPTH_IMAGE_PORT_NUM = SERVER_PORT_NUM + 2 

#connect to image hub
address= "tcp://"+HOST_IP+":"+IMAGE_HUB_PORT_NUM
IMAGE_SENDER = imagezmq.ImageSender(connect_to=address)

#connect to depth_image hub
address= "tcp://"+HOST_IP+":"+DEPTH_IMAGE_PORT_NUM 
DEPTH_IMAGE_SENDER = imagezmq.ImageSender(connect_to=address)

#connect to server
sock= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#connect to server
sock.connect((HOST_IP,SERVER_PORT_NUM))
#send server some of the camera details
data= json.dumps({"depth_scale":depth_scale,"width":w,"height":h ,"frame_rate": FRAME_RATE,"rate":RATE,"fx":fx,"fy":fy,"cx":cx,"cy":cy,"dist_coeffs":dist_coeffs})
sock.send(data.encode())

#return camera information to sensor
print("Sent data!");
#start thread for listen for request for camera info from server



<<<<<<< HEAD
def sendTriDepth():
    data=sock.recv(buffer_size) #expect to recieve array of indices to return depth value found in depth_image array
    jsonData= json.loads(data.decode())
       
    x1= jsonData.get("x")
    y1= jsonData.get("y")
    
    depth_1=[]
    
    for i in range(len(x1)):
        depth_1.append(depth_image[y1[i],x1[i]])
    
    jsonData = json.dumps({"depth": depth_1},default=str) #send as strings cause ints are serializable
    sock.send(jsonData.encode())


def sendVODepth():
    data=sock.recv(buffer_size) #expect to recieve array of indices to return depth value found in depth_image array
    jsonData= json.loads(data.decode())
      
    
    x1= jsonData.get("x1")
    x2= jsonData.get("x2")
    y1= jsonData.get("y1")
    y2= jsonData.get("y2")
    
    
    depth_1=[]
    depth_2=[]
    for i in range(len(x1)):
            depth_1.append(depth_image[y1[i],x1[i]])
            depth_2.append(prev_depth_image[y2[i],x2[i]])
            
    jsonData = json.dumps({"depth_1": depth_1,"depth_2": depth_2},default=str) #send as strings cause ints are serializable
    sock.send(jsonData.encode())
#thread job
def socketListen():
    
    while True:
        data=sock.recv(buffer_size) #expect to recieve array of indices to return depth value found in depth_image array
        jsonData= json.loads(data.decode())
        
        #depending on what client sends do something differnt
        request = jsonData.get("Request")
        if(request=="Tri_Depth"):
            sendTriDepth()
        elif(request=="VO_Depth"):
            sendVODepth()

#start thread for listen for request for camera info from server
t = threading.Thread(target =socketListen)
t.daemon=True #die when main thread dies
t.start() # start thread
=======
>>>>>>> b12c4dd64860ebb73b87ff9c336598f9a3347fba

rpiName = socket.gethostname()


def SendDepthImage(depth_image):
     DEPTH_IMAGE_SENDER.send_image(rpiName,depth_image)

while True:    
    
    
    #wait for client to request data
#    data= clientSocket.recv(100)

    #get frames from camera
    frame=pipeline.wait_for_frames()
    aligned_frames = align.process(frame)
<<<<<<< HEAD
=======
    
>>>>>>> b12c4dd64860ebb73b87ff9c336598f9a3347fba
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not color_frame or not depth_frame:
        continue
    #convert images to numpy arrays to send over network

    

    depth_image= np.asanyarray(depth_frame.get_data())
    color_image= np.asanyarray(color_frame.get_data())
    
    #gray scale frame
    gray_image= cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    
    #
    d_thread = threading.Thread(target =SendDepthImage,args=[depth_image])
    d_thread.start() # start thread

    #send frame
<<<<<<< HEAD
    sender.send_image(rpiName,gray_image) 
    sender.send_image(rpiName,depth_image)
=======
    IMAGE_SENDER.send_image(rpiName,gray_image) 
   

    
>>>>>>> b12c4dd64860ebb73b87ff9c336598f9a3347fba


