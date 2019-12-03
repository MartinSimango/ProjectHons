import time
import cv2
import numpy as np
import random
import math
from cv2 import aruco

ROBOT_HEIGHT=20 #20 cm off the ground
markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
arucoSquareDimension= 0.065 #length of side of square on aruco marker
#lookup table which stores positions of landmarks
LANDMARKS={}
LANDMARKS[0]= [0,0,0]
LANDMARKS[1]= [-31.5,-16.7,0]
LANDMARKS[2]= [-71.5,-47.5,0]
LANDMARKS[3]= [0,0,0]
LANDMARKS[4]= [0,0,0]
LANDMARKS[5]= [0,0,0]
LANDMARKS[6]= [0,0,0]
LANDMARKS[7]= [0,0,0]


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

def findMarkers(image): #try finding objects for num_seconds seconds #try finding max objects

    #move funcionality to tryTriangulate
    object_corners= {}
    markerCorners,markerID,_ = aruco.detectMarkers(image,markerDictionary)
    
    if(markerID is not None):
        for i in range(len(markerID)):
            #draw the square around makers
            
            aruco.drawDetectedMarkers(image,markerCorners)
            current_id = markerID[i][0]
            correctedCorners=getCorners(markerCorners[i][0]) #correct order of corners to orientade correctly
            
            object_corners[current_id] = [correctedCorners]

        
    for id_s in object_corners.keys():
        if(not id_s in LANDMARKS.keys()):
            del object_corners[id_s]
    print("Number Objects found so far: ",len(object_corners))
    return object_corners

def getPoints(xs,ys,NUM_POINTS):
    retPoints=[]

  
    for _ in range(NUM_POINTS):
        if(xs[0]!=xs[1]):
            x=random.randrange(xs[0],xs[1])
        else:
            x=int((xs[0] +xs[1])/2) 
        if(ys[0]!=ys[1]):
            y=random.randrange(ys[0],ys[2])
        else:
            y=int((ys[0]+ys[2])/2)
        retPoints.append([x , y])
    
  
    return retPoints
def getDepthInfo(corners,depth_image,depth_scale): #retrieves depth info for marker
    
    NUM_POINTS=10
    #get the y coords

    xs=[]
    ys=[]
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
   
    points= getPoints(xs,ys,NUM_POINTS)
    depth=[]
    for i in range(len(points)):
        x= points[i][0]
        y= points[i][1]
        depth.append(depth_image[y,x])
  
    mid_x= int((xs[0] +xs[1])/2) 
    mid_y= int((ys[0]+ys[2])/2)
    dist=[]
    for i in range(len(depth)):
        if(int(depth[i]) != 0):
            dist.append(float(depth[i])*depth_scale*100)

             
    depth = 0
    if(len(dist)!=0):
        # depth= int(round(np.average(dist)))
        depth= np.average(dist)
     

    return mid_x,mid_y,depth

def getLandmarkDistances(landmarkCorners,depth_image,depth_scale):
    distances={}
    focals={}
    for id_s in landmarkCorners.keys():
      
      mid_x,mid_y,d=getDepthInfo(landmarkCorners[id_s][0],depth_image,depth_scale)
      TL_x= landmarkCorners[id_s][0][0][0] 
      TR_x= landmarkCorners[id_s][0][1][0]
      #print("cr", landmarkCorners[id_s][0])
      width=abs(TR_x-TL_x)
      
      focal= (width*d)/6.5 # focal length in cm's 
     
      #print("Focal:",focal)
      if(d!=0):
        distances[id_s]=(d,mid_x,mid_y)
        focals[id_s]=focal

      
    return distances,focals

#points=[point_1_12,point_2_12,point_1_13,point_2_13,point_1_23,point_2_23]

def findClosest(points):
    #find closest 3 points

    N= len(points)
    min_dist= None
    min_p1= None
    min_p2= None
    min_p3= None
    for i in range(N):
        for j in range(N):
            if(j==i or j==i+1):
                continue
            for k in range(N):
                if(k==j or k==j+1):
                    continue
                dist= np.linalg.norm(points[i]-points[j]) + \
                      np.linalg.norm(points[i]-points[k]) + \
                      np.linalg.norm(points[j]-points[k])


                p1= points[i]
                p2= points[j]
                p3= points[k]
                if(min_dist is None or dist<min_dist):
                    min_dist=dist
                    min_p1= p1
                    min_p2= p2
                    min_p3= p3
    return min_p1,min_p2,min_p3
    
           


def getIntersection(x0, y0, r0, x1, y1, r1): #get the intersection between two circles
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)

    # non intersecting
    if d > r0 + r1 :
        print("Not inter")
        return None
    # One circle within other
    if d < abs(r0-r1):
        print("Inside!")
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        print("Coincide")
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d

        return (x3, y3, x4, y4)

def trilateratePos(distanceToLandmarks):
    x=[]
    y=[]
    z=[]
    d=[]
    x_offsets=[]
    z_offsets=[]
    for id_s in distanceToLandmarks.keys():

        x.append(LANDMARKS[id_s][0])
        y.append(LANDMARKS[id_s][1])
        z.append(LANDMARKS[id_s][2])
        d.append(distanceToLandmarks[id_s][0])
        x_offsets.append(distanceToLandmarks[id_s][1])
        z_offsets.append(distanceToLandmarks[id_s][2])
 
    x_1 = x[0]
    y_1 = y[0]
    z_1 = z[0]
    d_1 = d[0]
    x_1_offset= x_offsets[0]
    x_1_offset= x_offsets[0]

    x_2 = x[1]
    y_2 = y[1]
    z_2 = z[1]
    d_2 = d[1]
    x_2_offset= x_offsets[1]
    x_2_offset= z_offsets[1]

    x_3 = x[2]
    y_3 = y[2]
    z_3 = z[2]
    d_3 = d[2]
    x_3_offset= x_offsets[2]
    z_3_offset= z_offsets[2]
    


    #find intersections of these 3 circles (dont need to worry about z because surface is flat)
    # (x−x_1)^2+(y−y_1)^2=d_1^2
    # (x−x_2)^2+(y−y_2)^2=d_2^2
    # (x−x_3)^2+(y−y_3)^2=d_3^2
    #where x,y will be coord of robot
    #calculate intersections between circle A and B
   
    #intersection between circle 1 and circle 2
 
    inter_1_2 = getIntersection(x_1,y_1,d_1,x_2,y_2,d_2)
    if(inter_1_2 is None): #no intersection found
     
        print("OFF_1_2")
        return None
    #intersection between circle 1 and circle 3
    inter_1_3 = getIntersection(x_1,y_1,d_1,x_3,y_3,d_3)
    if(inter_1_3 is None):
        print("OFF_1_3")
        return None
    #intersection between circle 2 and circle 3
    inter_2_3 = getIntersection(x_2,y_2,d_2,x_3,y_3,d_3)
    if(inter_2_3 is None):
        print("OFF_2_3")
        return None
    
    #get the intersection points
    px1_12,py1_12,px2_12,py2_12= inter_1_2
    px1_13,py1_13,px2_13,py2_13= inter_1_3
    px1_23,py1_23,px2_23,py2_23= inter_2_3
    
    point_1_12= np.array([px1_12,py1_12])
    point_2_12= np.array([px2_12,py2_12])

    point_1_13= np.array([px1_13,py1_13])
    point_2_13= np.array([px2_13,py2_13])

    point_1_23= np.array([px1_23,py1_23])
    point_2_23= np.array([px2_23,py2_23])
    
    #Closest point problem
    
    #find closest 3 points
    points=[point_1_12,point_2_12,point_1_13,point_2_13,point_1_23,point_2_23]
    p1,p2,p3=findClosest(points)
    # print("All points: ",points)
    # print()
  
    #take averages of points
    x= (p1[0]+p2[0]+p3[0])/3 
    y= (p1[1]+p2[1]+p3[1])/3
    print("POS:",x,y)
    #trilatration formula (can't use as measurements are not perfect so might not be intersection)
    # A = 2*x_2 - 2*x_1
    # B = 2*y_2 - 2*y_1
    # C = d_1**2 - d_2**2 - x_a1**2 + x_2**2 - y_1**2 + y_2**2
    # D = 2*x_3 - 2*x_2
    # E = 2*y_3 - 2*y_2
    # F = d_2**2 - d_3**2 - x_2**2 + x_3**2 - y_2**2 + y_3**2
    
    # x = (C*E - F*B) / (E*A - B*D)
    # y = (C*D - A*F) / (B*D - A*E)

    # print("Robot is at:",x,y)
    #now find orientaion of robot (which direction its facing)
    #distance to first landmark (see diagram)
    c= np.arccos(abs(x-x_1)/(d_1))
    a= (np.pi/2) - np.arccos(abs(x_1_offset)/d_1)
    angle = c-a
    print("Angle is",np.rad2deg(angle))
    # still need to check which quadrant it's in

    #return None
    angle=np.deg2rad(0)
    return (x,y,ROBOT_HEIGHT,angle) #robot is at ground level 




   
  