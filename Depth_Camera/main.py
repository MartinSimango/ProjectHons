
#import necessary libraries
#import pyrealsense2 as rs
#for image producing
#pi ip is 146.231.181.162
import cv2
import numpy as np
import imagezmq # for recieving images from client
import imutils
import sys # for selecting which port to use for server
import socket # for recieving information from the client about the camera i.e depth_scale camera_intrinsics and depths of keypoints
import json

buffer_size=4096 #max size of data sent
(X,Y,Z)= (0.0,0.0,0.0) #starting coords

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
image_width   = jsonData.get("image_width") 
image_height  = jsonData.get("image_height") 
frame_rate    = jsonData.get("frame_rate") 
rate          = jsonData.get("rate") 
fx            = jsonData.get("fx")
fy            = jsonData.get("fy")
cx            = jsonData.get("cx")
cy            = jsonData.get("cy")
#how quick you want the camera to look between frames

print("Received: ", jsonData)

#start recieving images


def calculateDirectionChanges(H):
     camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]) #form the camera matrix
     _,Rotations,_,_ = cv2.decomposeHomographyMat(H,camera_matrix) #ignore translation and normal matrices
     

     print(Rotations[0])
     print()
     #validate different rotation matrices


#code adapted https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html
def detectAndMatch(grayFrame, grayNextFrame): #detect key points in both frames and match
    MIN_MATCH_COUNT = 50
    #turn both frames into grayscale
    

    #create sift object
    sift = cv2.xfeatures2d.SIFT_create()
    #detect key points

    kp_1, des_1 = sift.detectAndCompute(grayFrame,None)
    kp_2, des_2 = sift.detectAndCompute(grayNextFrame,None)

    #img_1=cv2.drawKeypoints(grayFrame,kp_1,frame)
    #img_2=cv2.drawKeypoints(grayNextFrame,kp_1,nextFrame)

    img_1= grayFrame
    img_2= grayNextFrame
    #now match the key points

    #can also try using Flanned Based matcher
    '''
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)
    '''
    bf = cv2.BFMatcher() 
    matches = bf.knnMatch(des_1,des_2, k=2)
    #sort the matches from best to worst
    #matches = sorted(matches, key=lambda x:x.distance)
    # test for good matches
    good = []   
    for m,n in matches:
        if m.distance < 0.70*n.distance:
            good.append(m)

    # cv2.drawMatchesKnn expects list of lists as matches.

    frame_keyPoints = None
    nextFrame_keyPoints = None
    matchesMask = None 
    if len(good)>MIN_MATCH_COUNT:
        #good= good[0:MIN_MATCH_COUNT]
        #only get the key points from within the good list
        frame_keyPoints = np.float32([ kp_1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        nextFrame_keyPoints = np.float32([ kp_2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
      
        #now find the Homography matrix finding the best set of inliers for the matches in
        H, mask = cv2.findHomography(frame_keyPoints, nextFrame_keyPoints, cv2.RANSAC,0.5)
        matchesMask = mask.ravel().tolist()
        calculateDirectionChanges(H) 
 

        #img_2 = cv2.polylines(img_2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        #h,w = img_1.shape
        #pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        #dst = cv2.perspectiveTransform(pts,M)
    #else: # must be more then 10 matches
       # print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
       #matchesMask = None
       
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
   
    #dra the matches
    img3 = cv2.drawMatches(img_1,kp_1,img_2,kp_2,good,None,**draw_params)
    #print("Kru", kp_1[0].pt)
    #print(matchesMask[0]))

    #img3 = cv2.drawMatchesKnn(img_1,kp_1,img_2,kp_2,good,flags=2,outImg=None)

    images = np.hstack((img_1, img_2))


    cv2.imshow("Frames",img3)
    #print(dst.shape)
    #cv2.imshow("ra",dst)
    
    #now get R and T from the homography
      
    cv2.waitKey(1)
    return (frame_keyPoints, nextFrame_keyPoints, matchesMask)









while True:
     
    rpiName,image= imageHub.recv_image() # get image from client
     
    imageHub.send_reply(b'OK')
    image= imutils.resize(image,width=image_width,height=image_height)
  
    
   
    if(count == rate): #past the first frame
        total_distance=0 
        distances_pos=[]
        distances_neg=[]
        kp_1,kp_2,matchesMask=detectAndMatch(image,prev_image)
        if(matchesMask is not None ):
            x1_s=[]
            x2_s=[]
            y1_s=[]
            y2_s=[]
            for i in range(len(matchesMask)):
                if(matchesMask[i]): #if a good match after and in RANSAC model

                    x_1=int(round(kp_1[i][0][0]))
                    y_1=int(round(kp_1[i][0][1]))
                
                    x_2=int(round(kp_2[i][0][0]))
                    y_2=int(round(kp_2[i][0][1]))
                    
                    x1_s.append(x_1)
                    x2_s.append(x_2)
                    y1_s.append(y_1)
                    y2_s.append(y_2)
                if(len(x1_s)==MAX_DEPTH_POINTS): #only get up to 20 matching points max
                    break
                    #send coords off to get back depth
            jsonData = json.dumps({"x1":x1_s,"y1":y1_s,"x2":x2_s,"y2":y2_s})       
            sock.send(jsonData.encode())
          
            #read depths_back
            data=sock.recv(buffer_size)
            jsonData= json.loads(data.decode())
            
            depth_1 = jsonData.get("depth_1")
            depth_2 = jsonData.get("depth_2")
            depth_len = len(depth_1)
        
            for i in range (depth_len):
                if(int(depth_1[i])==0 or int(depth_2[i])==0): #no depth for that point
                    continue
                dist=(int(depth_2[i])-int(depth_1[i]))*depth_scale*100  #get to cm
                if(dist>=0):
                    distances_pos.append(dist)
                else:
                    distances_neg.append(dist)
                #print("Key Points: ",kp_1[i],kp_2[i])  #check to see how similar key points are
                #print("Depth_1: ", int(depth_1[i])*depth_scale*100)
                #print("Depth_2: ", int(depth_2[i])*depth_scale*100)
              
               
           
            if(distances_pos): #some depth measurement
                total_distance=np.min(distances_pos)
                if(distances_neg):
                    if(abs(np.max(distances_neg))<total_distance):
                        total_distance=np.max(distances_neg)
             
                   
              
              
        #match points -
        #use ransac to get best inliers -
        # from homography matrix get rotation matrix and translation vectors (using decomposHomography)
        #compute points cloud for key points 
        #use points clouds to get rotation and translation matrices


    if(count%rate==0):
        prev_image=image.copy()
        #prev_depth_frame= depth_image.copy()
    if(count>rate):
        count = 0
        #print("%.6f" % total_distance)
  
    count= count+1



cv2.destroyAllWindows()
server_socket.close()

