# /dev/video0

from cv2 import *
import numpy as np

# initialize the camera

from uvc_tracker import Tracker 

cam = VideoCapture(0)   # 0 -> index of camera

cam.set(3, 1280)
cam.set(4, 720)
# _UK = np.array([
# 	[600.153387, 0, 315.459915], 
# 	[0, 598.015225, 222.933946], 
# 	[0,          0,          1]
# 	], np.float32)

# # USB Camera Distortion
# _UD = np.array([0.147084, -0.257330, 
# 	0.003032, -0.006975, 0.000000], np.float32)


tracker = Tracker(None, None, (9,6))
tracker.track()

def mouse_callback(event, x, y, flags, params):
	if event == 1:
		print(tracker.convert(x, y))

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




	    # gray = np.float32(cvtColor(img, COLOR_BGR2GRAY))
	    # # dst = cornerHarris(gray, 2, 1, 0.04)
	    # # dst = dilate(dst, None)

	    # corners = goodFeaturesToTrack(gray, 25, 0.01, 10)
	    # corners = np.int0(corners)
	    # for i in corners:
	    # 	x, y = i.ravel()
	    # 	circle(img, (x, y), 3, (0, 0, 255), -1)

	    # img[dst>0.01 * dst.max()] = [0, 0, 255]
	    # rectified_image = undistort(img, _UK, 
                    # _UD)
	    res, corners = findChessboardCorners(img, (9, 6), None, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK)
	    # if res:

	    if res:
		    print(res)

	    # 	print(corners, corners.shape)
	    imshow("cam-calibrate", img)
	    # imwrite('test.jpg', img)
	    # print(img.shape)
	    waitKey(1)
# destroyWindow("cam-calibrate")



