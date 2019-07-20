import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
#import pandas as pd


def createArucoMarkers():
    outputMarker=[]
    markerDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    for i in range(50):
        
        outputMarker=aruco.drawMarker(markerDictionary,i,500)
        imageName="4x4Marker_"+str(i)+".jpg"
        #f=open(imageName,"w")
        cv2.imwrite(imageName,outputMarker)

createArucoMarkers();



