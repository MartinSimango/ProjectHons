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
LANDMARKS[1]= [-29,-16.5,0]
LANDMARKS[2]= [-108.5,-72,0]
#LANDMARKS[3]= [-,-16.5,0]

LANDMARKS[4]= [-100,-105,0]
LANDMARKS[5]= [-114,-147,0]
LANDMARKS[6]= [-100,-118,0]
LANDMARKS[7]= [-62,-16.5,0]
LANDMARKS[8]= [-84,-36,0]  
LANDMARKS[9]= [-84,-191,0] 
LANDMARKS[10]=[-52,-205,0] 
LANDMARKS[11]=[6,-183,0] 
LANDMARKS[12]=[-21,-205,0]
LANDMARKS[13]=[29,-31,0] 
LANDMARKS[14]=[73,-60,0] 
LANDMARKS[15]=[84,-86,0] 


#DO 5,9 and 10 again

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

    ret_object_corners={}
    for id_s in object_corners.keys():
        if(id_s in LANDMARKS.keys()):
            ret_object_corners[id_s]=object_corners[id_s]
    #print("Number Objects found so far: ",len(object_corners),object_corners.keys())
    return ret_object_corners

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
      width=abs(TR_x-TL_x)
      
      focal= d/(6.5*width)#(width*d)/6.5 # focal length in cm's 
     
      #print("Focal:",focal)
      if(d!=0):
        distances[id_s]=(d,mid_x,mid_y)
        focals[id_s]=focal

      
    return distances,focals

#points=[point_1_12,point_2_12,point_1_13,point_2_13,point_1_23,point_2_23]
def getDistanceToClosestPointToCircle(point,center,r):
    magAC= np.linalg.norm(np.subtract(point,center))
    clos_x = center[0]+r*((point[0]-center[0])/magAC)
    clos_y = center[1]+r*((point[1]-center[1])/magAC)
    closeset_point=[clos_x,clos_y]
   
    return np.linalg.norm(np.subtract(point,closeset_point))
def findClosest(points,centers,radii):
    #find closest 3 points
    #first intersection points
    print(points)
    mins=[]
    mins_equal=[]
    j=0
    for i in [0,2,4] :
        diff_1=getDistanceToClosestPointToCircle(points[i],centers[j],radii[j])
        diff_2=getDistanceToClosestPointToCircle(points[i+1],centers[j],radii[j])
        # mag_1=np.linalg.norm(points[i])
        # mag_2=np.linalg.norm(points[i+1])
        # diff_1= abs(mag_1-radii[j])
        # diff_2= abs(mag_2-radii[j])
        j= j+1
        print("diffs:",diff_1,diff_2)
        if(abs(diff_1-diff_2)<0.001): #diff_1 and diff_2 are essentially the same
            mins_equal.append((points[i],points[i+1]))
            continue
        if(diff_1<diff_2):
            mins.append(points[i])
        else:
            mins.append(points[i+1])
       
    # if(len(mins)!=3):
    #     p= mins[0]
    #     for i in range(len(mins_equal)):
    #         mag_1=np.linalg.norm(np.subtract(mins_equal[i][0],p))
    #         mag_2=np.linalg.norm(np.subtract(mins_equal[i][1],p))
    #         if(mag_1<mag_2):
    #             mins.append(mins_equal[i][0])
    #         else:
    #             mins.append(mins_equal[i][1])


    min_p1,min_p2,min_p3=mins
    print("Mins: ",mins)


    # N= len(points)
    # min_dist= None
    # min_p1= None
    # min_p2= None
    # min_p3= None
    # for i in range(N):
    #     for j in range(N):
    #         if(j==i or j==i+1):
    #             continue
    #         for k in range(N):
    #             if(k==j or k==j+1):
    #                 continue
    #             dist= np.linalg.norm(abs(points[i])-abs(points[j])) + \
    #                   np.linalg.norm(abs(points[i])-abs(points[k])) + \
    #                   np.linalg.norm(abs(points[j])-abs(points[k]))


    #             p1= points[i]
    #             p2= points[j]
    #             p3= points[k]
    #             if(min_dist is None or dist<min_dist):
    #                 min_dist=dist
    #                 min_p1= p1
    #                 min_p2= p2
    #                 min_p3= p3
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
def findPoint(x_1,y_1,d_1,x_2,y_2,d_2,x_3,y_3,d_3):
    inter_1_2 = getIntersection(x_1,y_1,d_1,x_2,y_2,d_2)
    if(inter_1_2 is None): #no intersection found
     
        print("OFF_1_2")
        return None,None
    #intersection between circle 1 and circle 3
    inter_1_3 = getIntersection(x_1,y_1,d_1,x_3,y_3,d_3)
    if(inter_1_3 is None):
        print("OFF_1_3")
        return None,None
    #intersection between circle 2 and circle 3
    inter_2_3 = getIntersection(x_2,y_2,d_2,x_3,y_3,d_3)
    if(inter_2_3 is None):
        print("OFF_2_3")
        return None,None
    
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
    p1,p2,p3=findClosest(points,[[x_3,y_3],[x_2,y_2],[x_1,y_1]],[d_3,d_2,d_1])

    x= (p1[0]+p2[0]+p3[0])/3 
    y= (p1[1]+p2[1]+p3[1])/3
    return x,y
def calculateDistToLandmarkFromFurPoint(land_pos,furthest_point_dist):

    #shift coords so that robot pos is at the origin
    
    b= land_pos
    c= np.array([0,furthest_point_dist]) #furthest point
    
    BC= b-c
    BC_MAG= np.linalg.norm(BC)
    return BC_MAG
    
    # AB= a-b
    # AC= a-c
    # AC_mag= np.linalg.norm(AC) # should be the same as furthest_point
    # #print("Checking AC_MAG and fur_point_dist",AC_mag,furthest_point_dist)
    # AB_mag= np.linalg.norm(AB)
    # #find angle betweem AC and AB (in other words find angle between robot and landmark and robot and furthest point)
    # angle= np.arccos(np.dot(AB,AC)/(AB_mag*AC_mag))

    # #now use law of cosines to find BC 
    # BC= np.sqrt(AB_mag**2+AC_mag**2 - 2*AB_mag*AC_mag*np.cos(angle))
    # print("BC: ",BC)
    # print("OLD_BC",np.linalg.norm(b-c))
    #return BC
 
    

def trilateratePos(distanceToLandmarks,furthest_point_dist):
    x=[]
    y=[]
    z=[]
    d=[]
    d_2=[]
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
 
    x,y=findPoint(x_1,y_1,d_1,x_2,y_2,d_2,x_3,y_3,d_3)
    if(x is None):  #could not find the positon
        return None
   
    print("POS of robot:",x,y)
    #angle=np.deg2rad(0)
   # return (x,y,ROBOT_HEIGHT,angle) #robot is at ground level 


    #now compute furthest point coords using trilateration again 
    distances= []
    for id_s in distanceToLandmarks.keys():
        r_pos=np.array([x,y]) #robot pos
        
        l_x = distanceToLandmarks[id_s][1]
        l_y = np.sqrt(distanceToLandmarks[id_s][0]**2 - l_x**2 )
        
        land_pos=np.array([l_x,l_y]) #land_mark pos
        distances.append(calculateDistToLandmarkFromFurPoint(land_pos,furthest_point_dist))
    d_1 = distances[0]
    d_2 = distances[1]
    d_3 = distances[2]

    f_x,f_y=findPoint(x_1,y_1,d_1,x_2,y_2,d_2,x_3,y_3,d_3) #furthest point position
    if(f_x is None):
        return None 

    angle = np.arctan2((f_x-x),(f_y-y))
    #angle = np.arctan2((f_y-y),(f_x-x))
    #c= np.arccos(abs(x-x_1)/(d_1))
    #a= (np.pi/2) - np.arccos(abs(x_1_offset)/d_1)
    #angle = c-a
    print("Furthest point is at",f_x,f_y)
    print("Angle is",np.rad2deg(angle))
    # still need to check which quadrant it's in

    #return None
    #angle=angle+np.deg2rad(-4)
    return (x,y,ROBOT_HEIGHT,angle) #robot is at ground level 
    
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
  




   
  