# /dev/video0
import os, sys
from os.path import join as pjoin
path = os.path.abspath(os.getcwd()).rsplit('/')
rpath = '/'.join(path[: path.index('perls') + 1])
sys.path.append(pjoin(rpath, 'src/ros_'))

import cv2
import numpy as np

from utils.uvc_tracker import Tracker 

# initialize the camera
cam = cv2.VideoCapture(0)   # 0 -> index of camera
cam.set(3, 1280)
cam.set(4, 720)

_UK = np.array([
	[927.902447 ,0.000000 ,641.850659],
	[0.000000, 921.598756, 345.336021],
	[0.000000, 0.000000 ,1.000000]
	], dtype=np.float32)

_UD = np.array([
	0.078759, -0.143339, -0.000887 ,-0.001555 ,0.000000
	], dtype=np.float32)

# tracker = Tracker(None, None, K=_UK, D=_UD, board_size=(9,6))

def mouse_callback(event, x, y, flags, params):
	if event == 1:
		# print(tracker.convert(x, y))

		print(x, y)
while 1:
	s, img = cam.read()
	if s:    # frame captured without any errors
	    cv2.namedWindow("cam-calibrate", cv2.CV_WINDOW_AUTOSIZE)
	    cv2.setMouseCallback('cam-calibrate', mouse_callback)

	    res, corners = cv2.findChessboardCorners(img, (9, 6), 
	    	None, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)

	    if res:
		    print(res)

	    cv2.imshow("cam-calibrate", img)
	    cv2.waitKey(1)

# cv2.destroyWindow("cam-calibrate")



