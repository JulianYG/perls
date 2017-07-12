#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper 
"""

import pickle
import yaml
import numpy as np
from matplotlib import pyplot as plt

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

import rospy
import rosparam
import intera_interface
import time

sys.path.append(os.path.abspath('../'))
from robot import Robot

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


class KinectTracker():

	def __init__(self, robot, board_size=(2,2), itermat=(8, 9), 
		K=None, D=None,
		calib='../../../tools/calibration/calib_data/kinect'):

		if rospy.get_name() == '/unnamed':
			rospy.init_node('kinect_calibration')

		self._board_size = board_size
		self._checker_size = 0.0274

		self._arm = robot
		self._grid = itermat

		self._calib_directory = calib

		self.K = K
		self.d = D
		self.calibration_points = []
		self.image_points, self.gripper_points = [], []

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

		# Preparing...
		invRotation_dir = pjoin(self._calib_directory, 'kinect_inverseRotation.p')
		translation_dir = pjoin(self._calib_directory, 'kinect_translation.p')

		if os.path.exists(invRotation_dir) and os.path.exists(translation_dir):
			
			with open(invRotation_dir, 'rb') as f:
				ir = pickle.load(f)
			self.invR = ir

			with open(translation_dir, 'rb') as f:
				t = pickle.load(f)
			self.T = t
		
			return t, ir

		
		info = dict(
			board_size=self._board_size,
			directory=self._calib_directory
			)

		# After preparation is done, kickstart tracking!
		camera_man = rospy.Subscriber('/kinect2/hd/image_color', 
            Image, self.tracking_callback)

		try:
			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo('Shutting down robot camera corner detection')
			cv2.destroyWindow('kinect_calibrate')
			camera_man.unregister()
			print('computing...')
			self._compute()



	def tracking_callback(self, img_data):

		cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')

		# Match with pattern location

		detected, pix = self._detect_center(cv_image)

		# Skip if not found pattern
		if not detected:
			return

		# Get the real position
		gripper_pos = np.array(self._arm.get_tool_pose()[0], 
							   dtype=np.float32)

		self.image_points.append(pix)
		self.gripper_points.append(gripper_pos)
		print('{} image saved.'.format(len(self.image_points)))


	def _compute(self):

		object_points = self._get_object_points(len(self.image_points))

		# Solve for matrices
		retval, rvec, tvec = cv2.solvePnP(
			np.array(self.gripper_points, dtype=np.float32), 
			np.array(self.image_points, dtype=np.float32), 
			self.K, 
			self.d)

		print('Solving matrices...')

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

		print('Writing matrices...')

		self.T = tvec
		self.invR = invRotation

		return tvec, invRotation

	@staticmethod
	def convert(u, v, z, K, T, invR):

		# Pixel value
		p = np.array([u, v, 1], dtype=np.float32)

		tempMat = invR * K
		tempMat2 = tempMat.dot(p)
		tempMat3 = invR.dot(np.reshape(T, 3))

		# approx height of each block + 
		# (inv Rotation matrix * inv Camera matrix * point)
		# inv Rotation matrix * tvec
		s = (z + tempMat3[2]) / tempMat2[2]

		# s * [u,v,1] = M(R * [X,Y,Z] - t)  
		# ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
		temp = np.linalg.inv(K).dot(s * p)
		temp2 = temp - np.reshape(T, 3)
		gripper_coords = invR.dot(temp2)

		return gripper_coords

	def _detect_center(self, cv_img):
		
		# Look for pattern
		foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
			cv_img, self._board_size, None,
			cv2.CALIB_CB_ADAPTIVE_THRESH
		)

		cv2.drawChessboardCorners(cv_img, self._board_size, 
				usbCamCornerPoints, foundPattern)

		cv2.imshow('kinect_calibrate', cv_img)
		key = cv2.waitKey(1) & 0xff

		if key == 115:
			print('Recognizing pattern...')


			if not foundPattern:
				# print('Robot camera did not find pattern...'
				# 	' Skipping.')
				# cv2.imwrite(pjoin(self._calib_directory, 
				# 	'failures/{}.jpg'.format(rospy.Time.now())), cv_img)
				return False, None	

			else:
				cv2.cornerSubPix(cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY), 
						usbCamCornerPoints, (11, 11), (-1, -1), 
						(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

				usbCamCornerPoints = np.squeeze(usbCamCornerPoints)
				recognized_pix = np.mean(usbCamCornerPoints, axis=0)
			
				return True, recognized_pix

		return False, None

		
			

	# def match_eval(self):

	# 	def mouse_callback(event, x, y, flags, params):
	# 		if event == 1:
	# 			print((x, y), self.convert(x, y, 
	# 				self.z_offset, self.K, self.T, self.invR))
	# 	img = self._camera.snapshot()
	# 	while True:
	# 		try:
	# 			cv2.namedWindow('match_eval', cv2.CV_WINDOW_AUTOSIZE)
	# 			cv2.setMouseCallback('match_eval', mouse_callback)
	# 			# # Remove distortion
	# 			# rectified_image = cv2.undistort(img, self.K, 
	# 			# 	self.d)
	# 			cv2.imshow('match_eval', img)

	# 			# Update per millisecond
	# 			cv2.waitKey(1)
	# 		except KeyboardInterrupt:
	# 			cv2.destroyAllWindows()

if rospy.get_name() == '/unnamed':
	rospy.init_node('kinect_tracker')


limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, None)


dimension = (1920, 1080)
intrinsics = np.array([[1.0741796970429734e+03, 0., 9.3488214133804252e+02], 
                       [1.0640064260133906e+03, 0., 6.0428649994134821e+02], 
                       [0., 0., 1.]], dtype=np.float32)

distortion = np.array([ 3.4454149657654337e-02, 9.7555269933206498e-02,
       1.2981879029576470e-02, 2.7898744906562916e-04,
       -2.0379307133765162e-01 ], dtype=np.float32)

tracker = KinectTracker(robot, board_size= (9,6), itermat=(3,4), K=intrinsics,
	D=distortion)

# tracker.match_eval()
