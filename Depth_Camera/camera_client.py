import pyrealsense2 as rs #for accessing camera
import cv2
import numpy as np
import json #for sending neat forms of data
import sys
from imutils.video import VideoStream
import imagezmq
import time

#for threading
import threading
import socket
from queue import Queue

#some global variables
FRAME_RATE=15
HOST_IP= sys.argv[1]
PORT= sys.argv[2]
depth_image=[]

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


#start main thread server
address= "tcp://"+HOST_IP+":"+PORT
sender = imagezmq.ImageSender(connect_to=address)


def getDepthFromIndices(data):
    #todo

#thread job
def socketListen():
    #connect to server
    sock = socket.connect((HOST_IP,int(PORT)+1))
    #send server some of the camera details
    data= json.dumps({"depth_scale":depth_scale,"width":w,"height":h ,"frame_rate": FRAME_RATE})
    data.send(clientData.encode())
    #return camera information to sensor
    while True:
        data=sock.recv(1024) #expect to recieve array of indices to return depth value found in depth_image array
        #do somthing with that data t 
        data= getDepthFromIndices(data)

#start thread for listen for request for camera info from server
t = threading.Thread(target =socketListen)
t.daemon=True #die when main thread dies
t.start() # start thread



#server_socket=init_socket(int(sys.argv[1])) #get the socket number

#print("Server has started...")
#wait for client to connect
#clientSocket,address = server_socket.accept()
#print(f"Connection from {address} !") 
#send client some json
#clientData= json.dumps({"depth_scale":depth_scale,"width":w,"height":h })
#clientSocket.send(clientData.encode())

rpiName = socket.gethostname()

time.sleep(2.0)
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
    print(1)
    #gray scale frame
    gray_image= cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    
    #send frame
    sender.send_image(rpiName,gray_image) 
    #send depth info
    #sender.send_image(rpiName,depth_image[1:20])
    print("Sending data") 
    
    #now send the gray image
    #gray_image_string=str(gray_image.ravel().tolist())
    #depth_image_string=str(depth_image.ravel().tolist())
    
    #remove brackets from string and replace ',' with ''
    #gray_image_string=gray_image_string[1:len(gray_image_string)-1].replace(',','')
    #depth_image_string=gray_image_string[1:len(depth_image_string)-1].replace(',','')
    
 #   clientData= json.dumps({"image":gray_image.tolist(),"depth_matrix":depth_image.tolist()})
  #  clientSocket.send(clientData.encode())


