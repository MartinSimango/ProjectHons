from imutils import paths
import numpy as np
import imutils
import cv2

def find_marker(image):
    #conver image to grayscale and detect edgees
    gray = cv2.cvtColor(image,cv2.COLOR_BAYER_BG2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    edged = cv2.Canny(gray,35,125)

    cnts= cv2.findContours(edged.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cnts= imutils.grab_contours(cnts)
    c= max(cnts,key=cv2.contourArea)

    return cv2.minAreaRect(c)

#knownWidth is width of marker and perWidth is percieved width of 
#marker in the image
def distance_to_camera(knownWidth,focalLength,perWidth):
        return (knownWidth * focalLength)/ perWidth

KNOWN_DISTANCE = 24.0  #distance from camera to marker

KNOWN_WIDTH = 11.0 # width of marker

image = cv2.imread("pic")
#undistort image
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

