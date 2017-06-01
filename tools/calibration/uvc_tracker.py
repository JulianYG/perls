#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper 
"""

import pickle
import yaml
import numpy as np

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

import rospy
import rosparam
import intera_interface

from camera import UVCCamera
from ..src.ros_.utils.robot import Robot

rospy.init_node('track')

# USB Camera matrix 
_UK = np.array([
	[600.153387, 0, 315.459915], 
	[0, 598.015225, 222.933946], 
	[0,          0,          1]
	], np.float32)

# USB Camera Distortion
_UD = np.array([0.147084, -0.257330, 
	0.003032, -0.006975, 0.000000], np.float32)

class Tracker():

	def __init__(self, camera, robot, board_size=(2,2), itermat=(8, 9),
		calib='../calib_data'):

		self._camera = camera
		self._camera.turn_on()

		self._board_size = board_size

		self._arm = robot
		self._grid = itermat

		self._calib_directory = calib


	def track(self):

		origin = np.array([0.39, 0.26, 0.3], dtype=np.float32)

		calibration_grid = np.zeros(self._grid, np.float32)
		calibration_grid[:, :2] = np.mgrid[0: self._grid[0], 
										   0: self._grid[1]].T.reshape(-1, 2) * 0.07
		# And randomness to z								   
		calibration_grid[:, -1] += np.random.uniform(-0.2, 0.2, 
			self._grid[0] * self._grid[1])

		image_points, gripper_points = [], []

		for pos in calibration_grid:

			# Set position to reach for
			target = origin + pos
			end_state = dict(position=tuple(target),
						 orientation=(0.707, 0, 0.707, 0))
			# Move to target position
			self._arm.reach_absolute(end_state)

			# Wait till reach
			time.sleep(4)

			# Get the real position
			gripper_pos = np.array(self._arm.get_tool_pose()[0], 
								   dtype=np.float32)
			# Match with pattern location
			detected, pix = self._detect_center()

			# Skip if not found pattern
			if not detected:
				continue

			image_points.append(pix)
			gripper_points.append(gripper_pos)

		# Solve for matrices
		retval, rvec, tvec = cv2.solvePnP(
            np.array(gripper_points, dtype=np.float32), 
            np.array(image_points, dtype=np.float32), 
            self._camera.intrinsics, 
            self._camera.distortion)

        rotMatrix = cv2.Rodrigues(rvec)[0]
        invRotation = np.linalg.inv(rotMatrix)

        print(tvec, 'T')
        print(invRotation, 'iR')
        	
        data = dict(inverseRotation=invRotation,
        			translation=tvec)

        for name, mat in data.items():
			with open(pjoin(self._calib_directory, 
				'{}_{}.p'.format(self.__class__.__name__, name)), 'wb') as f:
				pickle.dump(mat, f)

        self.T = tvec
        self.invR = invRotation

        return tvec, invRotation


    def convert(self, u, v):

    	# Pixel value
    	p = np.array([u, v, 1], dtype=np.float32)

    	tempMat = self.invR * self._camera.intrinsics

        tempMat2 = tempMat.dot(p)
        tempMat3 = self.invR.dot(np.reshape(self.T, 3))

        # approx height of each block + 
        # (inv Rotation matrix * inv Camera matrix * point)
        # inv Rotation matrix * tvec
        s = (.026 + tempMat3[2]) / tempMat2[2]

        # s * [u,v,1] = M(R * [X,Y,Z] - t)  
        # ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
        temp = np.linalg.inv(self._camera.intrinsics).dot(s * p)
        temp2 = temp - np.reshape(self.T, 3)
        gripper_coords = self.invR.dot(temp2)

        return gripper_coords

    def _detect_center(self):

    	# Take picture
    	img = self._camera.snapshot()

    	# Look for pattern
    	foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
            img, self._board_size, None, 
            cv2.CALIB_CB_ADAPTIVE_THRESH
        )
        if not foundPattern:
            print('USB camera did not find pattern...')
            return False, None

        recognized_pix = np.mean(usbCamCornerPoints, axis=1)

        return True, recognized_pix

	def match_eval(self):

		def mouse_callback(event, x, y, flags, params):
            
            print(self.convert(x, y))

        while True:
            try:
                img = self._camera.snapshot()
                cv2.namedWindow('match_eval', cv2.CV_WINDOW_AUTOSIZE)
                cv2.setMouseCallback('match_eval', mouse_callback)

                # Remove distortion
                rectified_image = cv2.undistort(self._camera.intrinsics, 
                    self._camera.distortion)
                cv2.imshow('match_eval', rectified_image)
                # Update per millisecond
                cv2.waitKey(1)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()


robot = Robot(intera_interface.Limb('right'), 
	intera_interface.Gripper('right'))

camera = UVCCamera(0, (640, 480), _UK, _UD)


tracker = Tracker(camera, robot, 
	board_size=(2,2), itermat=(3, 2))

t, ir = tracker.track()

tracker.match_eval()




