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
import time

from camera import UVCCamera

sys.path.append(pjoin(os.getcwd(), '../../src/ros_/'))
from robot import Robot

from matplotlib import pyplot as plt

# USB Camera matrix 
# dimension (640, 480)
# _UK = np.array([
# 	[600.153387, 0, 315.459915], 
# 	[0, 598.015225, 222.933946], 
# 	[0,          0,          1]
# 	], np.float32)

# # USB Camera Distortion
# _UD = np.array([0.147084, -0.257330, 
# 	0.003032, -0.006975, 0.000000], np.float32)

# Dimension (1280, 720)
_UK = np.array([
	[927.902447 ,0.000000 ,641.850659],
	[0.000000, 921.598756, 345.336021],
	[0.000000, 0.000000 ,1.000000]
	], dtype=np.float32)

_UD = np.array([
	0.078759, -0.143339, -0.000887 ,-0.001555 ,0.000000
	], dtype=np.float32)

class Tracker():

	def __init__(self, camera, robot, board_size=(2,2), itermat=(8, 9),
		z=0, K=None, D=None, debug=False, calib='../calib_data'):

		self._camera = camera

		self._board_size = board_size
		self._checker_size = 0.027

		self.z_offset = z

		self._arm = robot
		self._grid = itermat

		self._calib_directory = calib

		self.debug = debug

		self.K = K
		self.d = D
		self.calibration_points = []

		self.track()

	def _get_object_points(self, size):
		"""
		Return a numpy array of object points in shape [num, 3], float32
		The object points represent the corner points on checkerboard
		in real world frame, origin from the top left corner of 
		the checkerboard
		"""
		raw_points = np.zeros(
			(self._board_size[1], self._board_size[0], 3), 
			dtype=np.float32)

		for i in range(self._board_size[1]):
			for j in range(self._board_size[0]):
				# Use 0 for z as for lying on the plane
				raw_points[i, j] = [i * self._checker_size, 
					j * self._checker_size, 0.]

		object_points = np.reshape(raw_points, 
			(self._board_size[0] * self._board_size[1], 3)).astype(np.float32)

		# Correspond correct number of points with sampled points
		return [object_points] * size

	def track(self):

		invRotation_dir = pjoin(self._calib_directory, 'Tracker_inverseRotation.p')
		translation_dir = pjoin(self._calib_directory, 'Tracker_translation.p')

		if os.path.exists(invRotation_dir) and os.path.exists(translation_dir):
			
			with open(invRotation_dir, 'rb') as f:
				ir = pickle.load(f)
			self.invR = ir

			with open(translation_dir, 'rb') as f:
				t = pickle.load(f)
			self.T = t
		
			return t, ir

		origin = np.array([0.3690, -0.2566, 0.27], dtype=np.float32)
		orn = np.array([0, 0, 0, 1], dtype=np.float32)

		calibration_grid = np.zeros((self._grid[0] * self._grid[1], 3), np.float32)
		calibration_grid[:, :2] = np.mgrid[0: self._grid[0], 
										   0: self._grid[1]].T.reshape(-1, 2) * 0.033
		# And randomness to z								   
		calibration_grid[:, -1] += np.random.uniform(-0.08, 0.2, 
			self._grid[0] * self._grid[1])

		image_points, gripper_points = [], []

		for pos in calibration_grid:

			# Set position to reach for
			target = origin + pos

			# Add randomness to orientation
			# target_orn = orn + np.random.normal(0, 0.05, 4)
			# target_orn /= np.sqrt(np.sum(target_orn ** 2))

			end_state = dict(position=tuple(target),
						 orientation=(0,0,0,1))
			# Move to target position
			self._arm.reach_absolute(end_state)

			# Wait till stabilize
			time.sleep(2)

			# Get the real position
			gripper_pos = np.array(self._arm.get_tool_pose()[0], 
								   dtype=np.float32)
			# Match with pattern location
			detected, pix = self._detect_center()

			# Skip if not found pattern
			if not detected:
				print('Pattern not detected. Skipping')
				continue

			image_points.append(pix)
			gripper_points.append(gripper_pos)

		object_points = self._get_object_points(len(image_points))

		if self.K is None or self.d is None:
			_, self.K, self.d, _, _ = cv2.calibrateCamera(
				object_points, self.calibration_points, (1280, 720))
			self.d = np.squeeze(self.d)

		# Solve for matrices
		retval, rvec, tvec = cv2.solvePnP(
			np.array(gripper_points, dtype=np.float32), 
			np.array(image_points, dtype=np.float32), 
			self.K, 
			self.d)

		rotMatrix = cv2.Rodrigues(rvec)[0]
		invRotation = np.linalg.inv(rotMatrix)
			
		data = dict(inverseRotation=invRotation,
					translation=tvec,
					intrinsics=self.K,
					distortion=self.d)

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

		tempMat = self.invR * self.K
		tempMat2 = tempMat.dot(p)
		tempMat3 = self.invR.dot(np.reshape(self.T, 3))

		# approx height of each block + 
		# (inv Rotation matrix * inv Camera matrix * point)
		# inv Rotation matrix * tvec
		s = (self.z_offset + tempMat3[2]) / tempMat2[2]

		# s * [u,v,1] = M(R * [X,Y,Z] - t)  
		# ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
		temp = np.linalg.inv(self.K).dot(s * p)
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
		
		if self.debug:
			plt.imshow(img)
			plt.show()	

		if foundPattern:
			cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 
					usbCamCornerPoints, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

			usbCamCornerPoints = np.squeeze(usbCamCornerPoints)
			recognized_pix = np.mean(usbCamCornerPoints, axis=0)

			cv2.drawChessboardCorners(img, self._board_size, 
				usbCamCornerPoints, foundPattern)
			self.calibration_points.append(usbCamCornerPoints)
			return True, recognized_pix
		else:
			return False, None

	def match_eval(self):

		def mouse_callback(event, x, y, flags, params):
			if event == 1:
				print((x, y), self.convert(x, y))
		img = self._camera.snapshot()
		while True:
			try:
				cv2.namedWindow('match_eval', cv2.CV_WINDOW_AUTOSIZE)
				cv2.setMouseCallback('match_eval', mouse_callback)
				# # Remove distortion
				# rectified_image = cv2.undistort(img, self.K, 
				# 	self.d)
				cv2.imshow('match_eval', img)

				# Update per millisecond
				cv2.waitKey(1)
			except KeyboardInterrupt:
				cv2.destroyAllWindows()

# rospy.init_node('track')
# limb = intera_interface.Limb('right')
# limb.set_joint_position_speed(0.2)
# robot = Robot(limb, None)

# camera = UVCCamera(0, (1280, 720), _UK, _UD)

# tracker = Tracker(camera, robot, 
# 	K=_UK, D=_UD, board_size=(9, 6), itermat=(9, 9), debug=False)

# tracker.match_eval()




