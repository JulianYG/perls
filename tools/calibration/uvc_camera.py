# /dev/video0

from cv2 import *
import numpy as np

# initialize the camera

from uvc_tracker import Tracker 

cam = VideoCapture(0)   # 0 -> index of camera

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

tracker = Tracker(None, None, K=_UK, D=_UD, board_size=(9,6))

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



