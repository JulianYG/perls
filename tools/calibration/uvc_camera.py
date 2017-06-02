# /dev/video0

from cv2 import *
import numpy as np

# initialize the camera

from uvc_tracker import Tracker 

cam = VideoCapture(0)   # 0 -> index of camera

cam.set(3, 1280)
cam.set(4, 720)

tracker = Tracker(None, None, (9,6))

def mouse_callback(event, x, y, flags, params):
	if event == 1:
		print(tracker.convert(x, y))

while 1:
	s, img = cam.read()
	if s:    # frame captured without any errors
	    namedWindow("cam-calibrate", CV_WINDOW_AUTOSIZE)
	    setMouseCallback('cam-calibrate', mouse_callback)

	    res, corners = findChessboardCorners(img, (9, 6), 
	    	None, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK)

	    if res:
		    print(res)

	    imshow("cam-calibrate", img)
	    waitKey(1)

# destroyWindow("cam-calibrate")



