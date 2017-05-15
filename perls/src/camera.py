# /dev/video0

from cv2 import *
import numpy as np

# initialize the camera
cam = VideoCapture(0)   # 0 -> index of camera


def mouse_callback(event, x, y, flags, params):
	if event == 1:
		print(x, y)

# P = np.array([[612.177795, 0.000000, 311.334067, 0.000000],
# 	[0.000000, 612.668945, 223.525080, 0.000000],
# 	[0.000000, 0.000000, 1.000000, 0.000000]])
# R, t = decomposeProjectionMatrix(P)[1: 3]
# print(R, t / t[3])
while 1:
	s, img = cam.read()
	if s:    # frame captured without any errors
	    namedWindow("cam-calibrate", CV_WINDOW_AUTOSIZE)
	    setMouseCallback('cam-calibrate', mouse_callback)
	    # res, corners = findChessboardCorners(img, (9, 6), None, CALIB_CB_ADAPTIVE_THRESH)
	    # if res:
	    	# print(corners, corners.shape)
	    imshow("cam-calibrate",img)
	    # imwrite('test.jpg', img)
	    # print(img.shape)
	    waitKey(0)
# destroyWindow("cam-calibrate")



