
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


#socket for requesting camera information that is not pictures
def init_socket(port_num):
    #CREATE TCP SOCKET
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind socket
    server_socket.bind((socket.gethostbyname(),port_num))

    #start listening
    server_socket.listen(5) 
    return server_socket
    


#code adapted https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html
def detectAndMatch(grayFrame, grayNextFrame): #detect key points in both frames and match
    MIN_MATCH_COUNT = 10
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
        #only get the key points from within the good list
        frame_keyPoints = np.float32([ kp_1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        nextFrame_keyPoints = np.float32([ kp_2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
      
        #now find the Homography matrix finding the best set of inliers for the matches in
        M, mask = cv2.findHomography(frame_keyPoints, nextFrame_keyPoints, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()


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



prev_image = None
prev_depth_frame = None
count=0
total_distance = 0
RATE=5 #how quick you want the camera to look between frames
#create image hub and start start

PORT= "tcp://*:"+sys.argv[1]
imageHub= imagezmq.ImageHub(open_port=PORT) #equivalent to sockets but for recieving images
server_socket= init_socket(int(sys.argv[1])+1) #start socket for listening to input for camera information

print("Server started!")

while True:
     
    rpiName,image= imageHub.recv_image() # get image from client
     
    imageHub.send_reply(b'OK')
    image= imutils.resize(image,width=400,height=600)
  
    

    if(count >RATE): #past the first frame
        kp_1,kp_2,matchesMask=detectAndMatch(image,prev_image)
        if(matchesMask is not None ):
            distances=[]
        for i in range(len(matchesMask)):
            if(matchesMask[i]):
                x_1=int(kp_1[i][0][0])
                y_1=int(kp_1[i][0][1])
                
                x_2=int(kp_2[i][0][0])
                y_2=int(kp_2[i][0][1])

                #print(x_1,y_1)
                depth_1 = 0;#depth_image[y_1,x_1].astype(float)
                depth_2 = 0# prev_depth_frame[y_2,x_2].astype(float)

                distance = 0#(depth_1 * depth_scale) - (depth_2 *depth_scale)
                distances.append(distance)
        total_distance = np.average(distances) 
        #match points -
        #use ransac to get best inliers -
        # from homography matrix get rotation matrix and translation vectors (using decomposHomography)
        #compute points cloud for key points 
        #use points clouds to get rotation and translation matrices


    if(count%RATE==0):
        prev_image=image.copy()
        #prev_depth_frame= depth_image.copy()
    
    #print(total_distance)
    count= count+1



cv2.destroyAllWindows()
server_socket.close()
    # for y in range(480):
     #       for x in range(640):
      #          dist = depth_frame.get_distance(x, y)
       #         print(dist)

