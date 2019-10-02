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
depth_profile=rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics=depth_profile.get_intrinsics()
w,h= depth_intrinsics.width, depth_intrinsics.height

#get depth scale
depth_sensor= profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

#get camera matrix details
fx= depth_intrinsics.fx
fy= depth_intrinsics.fy
cx= depth_intrinsics.ppx
cy= depth_intrinsics.ppy
dist_coeffs= depth_intrinsics.coeffs


#start main thread server
address= "tcp://"+HOST_IP+":"+PORT
sender = imagezmq.ImageSender(connect_to=address)


def getTriDepth():



def getVODepth():
    x1 = jsonData.get("x1")
    
    x1 = jsonData.get("x1")
#thread job
def socketListen():
    
    #create socket
    sock= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    #connect to server
    sock.connect((HOST_IP,int(PORT)+1))
    #send server some of the camera details
    data= json.dumps({"depth_scale":depth_scale,"width":w,"height":h ,"frame_rate": FRAME_RATE,"rate":RATE,"fx":fx,"fy":fy,"cx":cx,"cy":cy,"dist_coeffs":dist_coeffs})
    sock.send(data.encode())
    #return camera information to sensor
    print("Send data!");

    while True:
        data=sock.recv(buffer_size) #expect to recieve array of indices to return depth value found in depth_image array
        jsonData= json.loads(data.decode())
        
        #depending on what client sends do something differnt
        request = jsonData.get("Request")
        if(request=="Tri_Depth"):
            
        elif(request=="VO_Depth"):

        #do somthing with that data t 
        x1= jsonData.get("x")
       #x2= jsonData.get("x2")
        y1= jsonData.get("y")
       # y2= jsonData.get("y2")
        depth_1=[]
      #  depth_2=[]
        for i in range(len(x1)):
            depth_1.append(depth_image[y1[i],x1[i]])
            #depth_2.append(prev_depth_image[y2[i],x2[i]])
            
        jsonData = json.dumps({"depth": depth_1},default=str) #send as strings cause ints are serializable
        sock.send(jsonData.encode())
        #data= getDepthFromIndices(data)
#start thread for listen for request for camera info from server
t = threading.Thread(target =socketListen)
t.daemon=True #die when main thread dies
t.start() # start thread




rpiName = socket.gethostname()
count=0
#time.sleep(2.0)
while True:    
    
    
    #wait for client to request data
#    data= clientSocket.recv(100)

    #get frames from camera
        
    frame=pipeline.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    #convert images to numpy arrays to send over network
    
    depth_image= np.asanyarray(depth_frame.get_data())
    color_image= np.asanyarray(color_frame.get_data())
    #gray scale frame
    gray_image= cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    
    #send frame
    sender.send_image(rpiName,gray_image) 
    if(count%RATE==0 ):
        prev_depth_image=depth_image.copy()
    if(count>RATE):
        count=0
    count= count+1
    


