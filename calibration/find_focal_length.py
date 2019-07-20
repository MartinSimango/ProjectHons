KNOWN_DISTANCE = 24.0  #distance from camera to marker

KNOWN_WIDTH = 11.0 # width of marker

#image = cv2.imread("pic")
#undistort image
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
