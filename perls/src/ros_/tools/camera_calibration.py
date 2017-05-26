#!/usr/bin/env python
from __future__ import print_function

import pickle
import yaml
import numpy as np

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf
import rospy
import rosparam
import intera_interface
from std_msgs.msg import String, Header
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
	Vector3Stamped,
	Vector3
)


class CameraCalibrator(object):
	"""
	The basic camera calibrator parent class. Takes the 
	standard point pattern size of checkerboard (default 9x6) 
	and checker size in meters, as well as dimension of the 
	camera. (UVC 640 x 480)
	Calib_min: Minimum number of different angles (perspective)
	required to accurately calibrate the camera. Typically 4 - 7.
	The calibrator will save the intrinsics and distortion of the
	camera, as well as the rotation / translation for each 
	displayed checkerboard. It also outputs a meta file .yml 
	about calibration info
	"""
	def __init__(self, camera, boardSize, checkerSize, 
		directory, calib_min):

		self._camera = camera

		self._board_size = boardSize
		self._checker_size = checkerSize

		self._calib_directory = directory
		self._calibration_points = calib_min

		self._num_corners = self._board_size[0] * self._board_size[1]

		# Initialize image points for consistency
		self.image_points = []

	def calibrate(self):
		"""
		Calibrate the camera if target information does not exist. 
		Otherwise read stored data.
		Returns the following calibration parameters:
		Camera intrinsic matrix,
		Camera distortion vector,
		Camera rotation matrix,
		Camera translation vector.
		If stereo calibration:
		Intrinsic matrix/distortion vector for both cameras, 
		as well as 
		Rotation / translation of camera 2 w.r.t camera 1
		Essential matrix
		Fundamental matrix
		"""
		meta_file = pjoin(self._calib_directory, 'meta.yml')
		if not os.path.exists(meta_file):
			return self._calibrate_camera()

		else:
			camera_type, info_list = self.parse_meta(meta_file)
			return self._read_params(camera_type, info_list)

	def _calibrate_camera(self):
		"""
		The standard calibration procedure using opencv2
		"""
		# Get sets of points
		object_points = self._get_object_points()
		self._get_image_points()

		# Crank the calibration
		retval, K, d, rvec, tvec = cv2.calibrateCamera(
			object_points, self.image_points, self._camera.dimension)

		self._write_params({
			'rotation': rvec,
			'translation': tvec,
			'instrinsic': K, 
			'distortion': d
			})

		self.write_meta({
			'calibration_': self.__name__,
			'data': 
				['rotation', 
				'translation', 
				'intrinsic',
				'distortion']
			})

		return K, d, rvec, tvec
   
	def _get_image_points(self):
		"""
		Get image points for calibratio; polling from
		the camera(s)
		"""
		raise NotImplementedError('Each calibrator class must re-implement this method.')

	def _get_object_points(self):
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
			(self._num_corners, 3)).astype(np.float32)

		# Correspond correct number of points with sampled points
		return [object_points] * self._calibration_points

	def parse_meta(self, file=''):
		"""
		Static method to parse the metadata file
		"""
		meta = file or pjoin(self._calib_directory, 'meta.yml')
		with open(meta, 'r') as f:
			data_info = yaml.load(f)

		camera = data_info['calibration_']
		names = data_info['data']
		return camera, names

	def write_meta(self, meta_data):
		"""
		Outputs a metadata file .yml
		"""
		with open(pjoin(self._calib_directory, 'meta.yml'), 'w') as f:
			yaml.dump(meta_data, f, default_flow_style=False)

	def _write_params(self, data):
		"""
		Given data, write into files.
		Data: a dictionary with (name, value) as key, value pairs
		"""
		for name, mat in data.items():
			with open(pjoin(self._calib_directory, 
				'{}_{}.p'.format(self.__class__.__name__, name)), 'wb') as f:
				pickle.dump(mat, f)

	def _read_params(self, camera, names):
		"""
		Names: a list of properties in query.
		Reads the data and returns corresponding stored data
		in the same order as given names.
		"""
		data = {}
		for name in names:
			with open(pjoin(self._calib_directory, 
				'{}_{}.p'.format(camera, name)), 'rb') as f:
				data[name] = pickle.load(f)
		return data


class MonocularCalibrator(CameraCalibrator):

	def __init__(self, camera, boardSize, 
		checkerSize, directory, calib_min=4):

		super(UVCCalibrator, self).__init__(camera, boardSize, 
			checkerSize, directory, calib_min)

	def _get_image_points(self):

		info = dict(
			board_size=self._board_size,
			num_of_points=self._num_corners,
			directory=self._calib_directory
			)

		img = self._camera.snapshot()

		# Sample enough image points
		while len(self.image_points) < self._calibration_points:
			img_pts = self._camera.callback(img, info)
			if img_pts != None:
				self.image_points.append(img_pts)


class RobotCalibrator(CameraCalibrator):
	"""
	camera: 'right_hand_camera' and 'head_camera' for sawyer
	"""
	def __init__(self, camera, boardSize, checkerSize, 
		directory, calib_min=4):

		super(RobotCalibrator, self).__init__(camera, 
			boardSize, checkerSize, directory, calib_min)

		self._camera_type = camera._type

	def _get_image_points(self):
		"""
		Use a callback function to process captured images
		"""
		info = dict(
			board_size=self._board_size,
			num_of_points=self._num_corners,
			directory=self._calib_directory,
			point_list=self.image_points,
			calibration_points=self._calibration_points
			)

		self._camera.start_streaming(self._camera_type)
		self._camera.set_callback(self._camera_type, 
			self._camera.callback,
			rectify_image=True, 
			callback_args=info)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo('Shutting down robot camera corner detection')
			self._camera.stop_streaming(self._camera_type)


class StereoCalibrator(CameraCalibrator):

	"""
	Stereo camera calibration. Requires the two cameras be 
	aligned on the same x-axis. 
	Both cameras are not robot cameras, and camera indices
	must be provided.
	Also assumes two cameras are identical; only requires
	one camera dimension input
	"""
	def __init__(self, stereoCamera, boardSize, 
		checkerSize, directory, calib_min=4):
		"""
		Note constructor here takes the index of 
		both left and right cameras
		"""

		self._camera = stereoCamera

		self.left_image_points = []
		self.right_image_points = [] # image_points

	def _calibrate_camera(self):

		object_points = self._get_object_points()
		self._get_image_points()

		#TODO: Use cornerSubPix
		retVal, k1, d1, k2, d2, R, T, E, F = cv2.stereoCalibrate(
			object_points, 
			self.left_image_points, 
			self.right_image_points, 

			self._camera_dim,

			flags=(
				cv2.CALIB_FIX_INTRINSIC + 
				cv2.CALIB_FIX_ASPECT_RATIO +
				cv2.CALIB_ZERO_TANGENT_DIST +
				cv2.CALIB_SAME_FOCAL_LENGTH +
				cv2.CALIB_RATIONAL_MODEL +
				cv2.CALIB_FIX_K3 + 
				cv2.CALIB_FIX_K4 + 
				cv2.CALIB_FIX_K5)
			)
		self._write_params({
			'left_intrinsic': k1,
			'left_distortion': d1,
			'right_intrinsic': k2,
			'right_distortion': d2, 
			'rotation': R,
			'translation': T,
			'essential': E, 
			'fundamental': F
			})

		self.write_meta({
			'calibration_': self.__name__,
			'data': 
				['left_intrinsic', 
				'left_distortion', 
				'right_distortion',
				'right_distortion',
				'rotation',
				'translation',
				'essential', 
				'fundamental'
				]
			})

		return k1, d1, k2, d2, R, T, E, F

	def _get_image_points(self):

		info = dict(
			board_size=self._board_size,
			num_of_points=self._num_corners,
			directory=self._calib_directory
			)

		left_img, right_img = self._camera.snapshot()

		# Sample enough image points
		while len(self.left_image_points) < self._calibration_points:
			left_pts, right_pts = self._camera.callback(left_img, right_img, info)
			if left_pts != None and right_pts != None:
				self.left_image_points.append(left_pts)
				self.right_image_points.append(right_pts)


class HybridCalibrator(StereoCalibrator):
	"""
	Hybrid camera calibration (generalization of stereo)
	By default, camera1 (left) is external camera (usually providing
	clear top view, can be Kinect or UVC); camera 2 is 
	the robot camera used for transforming camera coords 
	to the base coords (for gripper)

	Performs calibration separately, then compute the 
	relation between two cameras
	"""

	def __init__(self, camera, boardSize, 
		checkerSize, directory, calib_min=7):
		"""
		Note the constructor takes the index of external 
		camera, and the string name of the robot camera (internal)
		"""
		super(HybridCalibrator, self).__init__(camera, boardSize, 
			checkerSize, directory, calib_min)

		self.tl = tf.TransformListener()
	

	def _get_image_points(self):
		"""
		Use a callback function to process captured images
		"""
		info = dict(
			board_size=self._board_size,
			num_of_points=self._num_corners,
			directory=self._calib_directory,
			left_point_list=self.left_image_points,
			right_point_list=self.right_image_points,
			calibration_points=self._calibration_points
			)

		robot_cam = self._camera._right_camera
		robot_cam.start_streaming(robot_cam._type)
		robot_cam.set_callback(robot_cam._type, 
			self._camera.callback,
			rectify_image=True, 
			callback_args=info)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo('Shutting down robot camera corner detection')
			robot_cam.stop_streaming(robot_cam._type)


	def _calibrate_camera(self):
		"""
		The standard calibration procedure using opencv2
		"""
		# Get sets of points
		object_points = self._get_object_points()
		
		self._get_image_points()

		objp = object_points[0]
		dimensions = self._camera.dimension

		external_to_internal = np.zeros((4, 4), dtype=np.float32)


		retval, k1, d1, r1, t1 = cv2.calibrateCamera(
			object_points, 
			self.left_image_points,
			dimensions[0]
			)

		retval, k2, d2, r2, t2 = cv2.calibrateCamera(
			object_points,
			self.right_image_points,

			# Check how to get idmension
			dimensions[1]
			)
	
		for i in range(self._calibration_points):

			r_external, _ = cv2.Rodrigues(r1[i])
			r_internal, _ = cv2.Rodrigues(r2[i])

			trans_external = np.vstack((np.column_stack((r_external, t1[i])), [0, 0, 0, 1]))
			trans_internal = np.vstack((np.column_stack((r_internal, t2[i])), [0, 0, 0, 1]))

			external_to_internal += trans_internal.dot(np.linalg.inv(trans_external))

		external_to_internal /= self._calibration_points		


		# for i in range(self._calibration_points):

		# 	rvecs_external, tvecs_external, _ = cv2.solvePnPRansac(objp, self.external_img_points[i], 
		# 		self._external_intrinsic, self._external_distortion)

		# 	rvecs_internal, tvecs_internal, _ = cv2.solvePnPRansac(objp, self.image_points[i], 
		# 		self._internal_intrinsic, self._internal_distortion)

		# 	r_external, _ = cv2.Rodrigues(rvecs_external)
		# 	r_internal, _ = cv2.Rodrigues(rvecs_internal)

		# 	trans_external = np.vstack((np.column_stack((r_external, tvecs_external)), [0, 0, 0, 1]))
		# 	trans_internal = np.vstack((np.column_stack((r_internal, tvecs_internal)), [0, 0, 0, 1]))

		# 	external_to_internal += trans_internal.dot(np.linalg.inv(trans_external))

		# external_to_internal /= self._calibration_points

		robot_cam_type = self._camera._right_camera._type

		if self.tl.frameExists(robot_cam_type) and self.tl.frameExists('base'):

			t = self.tl.getLatestCommonTime(robot_cam_type, 'base')
			hdr = Header(stamp=t, frame_id=robot_cam_type)
			translation, rotation = self.tl.lookupTransform('base', robot_cam_type, t)

			mat = tf.TransformerROS().fromTranslationRotation(translation, rotation)
			TR = mat.dot(external_to_internal)
	
			self._write_params({
				'transform': TR
				'external_intrinsic': k1,
				'external_distortion': d1,
				'internal_intrinsic': k2,
				'internal_distortion': d2
				})

			self.write_meta({
				'calibration_': 'DuoCalibrator',
				'data': 
					[
					'transform',
					'external_intrinsic',
					'external_distortion',
					'internal_intrinsic',
					'internal_distortion'
					]
				})

			# print(k1, k2, d1, d2)
			return TR, k1, k2, d1, d2

		else:
			raise Exception('Frame does not exist. Severe Error!')

	


