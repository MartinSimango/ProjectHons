
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
import math 










def calculateDirectionChanges(kp_1,kp_2,matchesMask,kp,prev_kp,num_points):   

    #see which kp_1 are in kp and which kp_2 are in prev_kp
    #just take 5 points
    x_change = []
    z_change = []
    count=0
    kp_new={}
    prev_kp_new={}
    for point in kp.keys():
        x= int(round(point.pt[0]))
        y= int(round(point.pt[1]))
        kp_new[(x,y)] = kp[point]
    for point in prev_kp.keys():
        x= int(round(point.pt[0]))
        y= int(round(point.pt[1]))
        prev_kp_new[(x,y)] = prev_kp[point]
    print(len(kp_1))
    for i in range(len(kp_1)): 
        if(matchesMask[i]):
           
            x_1=int(round(kp_1[i][0][0]))
            x_2=int(round(kp_2[i][0][0]))
            z_1=int(round(kp_1[i][0][1]))
            z_2=int(round(kp_2[i][0][1]))
          
            #if points are very close then just
            if( (x_1,z_1) in kp_new.keys() and (x_2,z_2) in prev_kp_new.keys()):
                print("YES!")
                pos_1=kp_new[(x_1,z_1)] # formatta as ([0,1],z),
                x_1= pos_1[0][0]
                z_1= pos_1[0][1]
                pos_2=prev_kp_new[(x_2,z_2)]
                x_2= pos_2[0][0]
                z_2= pos_2[0][1]
                x_change.append(x_2-x_1)
                z_change.append(z_2-z_1)
                if(count== num_points):
                    break
                count= count +1

    if(len(x_change)==0):
        return (0,0)
    return int(round(np.average(x_change))), int(round(np.average(z_change)))
  
        

   
def getKeyPointsDepth(image,sock,num_key_points,socket_lock,buffer_size=4096):
    
    sift = cv2.xfeatures2d.SIFT_create()

    kp, _ = sift.detectAndCompute(image,None)
    
    num_p = min(num_key_points,len(kp))
    kp=kp[:num_p]  #only take a limited amount of key points
    #now 
    x_s=[]
    y_s=[]
    kp_ret=[]


    for i in range(len(kp)):
           
        x=int(round(kp[i].pt[0]))
        y=int(round(kp[i].pt[1]))

        kp_ret.append((x,y))
        x_s.append(x)
        y_s.append(y)
        
    # end for
    # not get depth for all x,y coords
    jsonData =  json.dumps({"Request": "Tri_Depth"}) #first tell client what you are requesting
    
    socket_lock.acquire()
    #print("VO thread has lock")
    
    sock.send(jsonData.encode())
    #now send of the request
    jsonData = json.dumps({"x":x_s,"y":y_s})   
    sock.send(jsonData.encode())
    
    #now measure distance change in x and y direction
    #x_change,y_change=calculateDirectionChanges(kp_1,kp_2,matchesMask)
    #read depths_back
    data=sock.recv(buffer_size)
    
    socket_lock.release()
    #print("VO thread has released lock")
    jsonData= json.loads(data.decode())
    
    depths= jsonData.get("depth") 
       
    depths=list(map(float,depths)) #convert string array to int array
    return kp, depths



#code adapted https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html
def detectAndMatch(grayFrame, grayNextFrame,kp_1,kp_2): #detect key points in both frames and match
    MIN_MATCH_COUNT = 5
    #turn both frames into grayscale
    

    #create sift object
    
    sift = cv2.xfeatures2d.SIFT_create()

    #detect key points
    
    #kp_1, des_1 =   sift.detectAndCompute(grayFrame,None)
    #kp_2, des_2 = sift.detectAndCompute(grayNextFrame,None)
    _,des_1 = sift.compute(grayFrame,kp_1)
    _,des_2 = sift.compute(grayNextFrame,kp_2)
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
    
    #return values
    frame_keyPoints = []
    nextFrame_keyPoints = []
    matchesMask = []
    #H=None
    if len(good)>MIN_MATCH_COUNT:
        #good= good[0:MIN_MATCH_COUNT]
        #only get the key points from within the good list
        frame_keyPoints = np.float32([ kp_1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        nextFrame_keyPoints = np.float32([ kp_2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
      
        #now find the Homography matrix finding the best set of inliers for the matches in
        H, mask = cv2.findHomography(frame_keyPoints, nextFrame_keyPoints, cv2.RANSAC,0.5)
        matchesMask = mask.ravel().tolist()
        #camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]) #form the camera matrix

    #drawing matching features  
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
            #now measure distance change in x and y direction
            #x_change,y_change=calculateDirectionChanges(kp_1,kp_2,matchesMask)
   
    #draw the matches
   # img3 = cv2.drawMatches(img_1,kp_1,img_2,kp_2,good,None,**draw_params)
    #print("Kru", kp_1[0].pt)
    #print(matchesMask[0]))

    #img3 = cv2.drawMatchesKnn(img_1,kp_1,img_2,kp_2,good,flags=2,outImg=None)

    #images = np.hstack((img_1, img_2))
            


    #cv2.imshow("Frames",images)
   
    #have 
    #cv2.waitKey(15)
    return (frame_keyPoints, nextFrame_keyPoints, matchesMask)




def getPositionFromKeyPoints(sock,kp_1,kp_2,matchesMask,depth_scale,fx,VO_X,VO_Z,buffer_size=4096): #max size of data sent: # after some calculate returns the position of the camera
     MAX_DEPTH_POINTS=50 # how many depth points we want to ask for
     depth_KP= {} #dictionary to keep depths of key points
     if(matchesMask is not None ):
            x1_s=[]
            x2_s=[]
            y1_s=[]
            y2_s=[]
            keyPoint_indices=[]
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
                    keyPoint_indices.append(i)
                if(len(x1_s)==MAX_DEPTH_POINTS): #only get up to 20 matching points max
                    break
                    #send coords off to get back depth
            # end for
            # not get depth for all x,y coords
            jsonData =  json.dumps({"Request": "VO_Depth"}) #first tell client what you are requesting
            sock.send(jsonData.encode())
            #now send of the request
            jsonData = json.dumps({"x1":x1_s,"y1":y1_s,"x2":x2_s,"y2":y2_s})       
            sock.send(jsonData.encode())
          
            #now measure distance change in x and y direction
            #x_change,y_change=calculateDirectionChanges(kp_1,kp_2,matchesMask)
            #read depths_back
            data=sock.recv(buffer_size)
            jsonData= json.loads(data.decode())
            
            depth_1 = jsonData.get("depth_1")
            depth_2 = jsonData.get("depth_2")
            
            #get change in the x direction
            x_change = int(calculateDirectionChanges(kp_1,kp_2,keyPoint_indices,depth_2,depth_scale,fx))
            
            #now calculate the depth change in z direction
            depth_len = len(depth_1)
            distances_pos=[]
            distances_neg=[]
            total_distance=0
            for i in range (depth_len):
                if(int(depth_1[i])==0 or int(depth_2[i])==0): #no depth for that point
                    continue
                dist=(int(depth_2[i])-int(depth_1[i]))*depth_scale*100  #get to cm
                if(dist>=0):
                    distances_pos.append(dist)
                else:
                    distances_neg.append(dist)
               
            if(len(distances_pos)>len(distances_neg)):
                total_distance = np.min(distances_pos)
            else: 
                if(distances_neg):
                    total_distance = np.max(distances_neg)
              
            mul=1
            if(int(total_distance)<0):
                mul=-1

            z_change =  np.square(int(total_distance) -int(x_change)) *mul

            #print("Z_change",z_change)
            VO_X = VO_X + x_change 
            VO_Z = VO_Z + z_change
            return (VO_X,VO_Z)
               
           
                   







