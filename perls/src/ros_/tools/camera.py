#!/usr/bin/env python

"""
A general set of classes of cameras; instances are 
fed to calibration classes.
"""
from __future__ import print_function

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

class Camera(object):
	"""
	The basic camera class. Provides a convenient basic 
	wrapper interface that can be called from calibrators.
	"""

	def __init__(self, camera, dimension, 
		intrinsics, distortion):

		# Due to different type of camera 
		# interfaces, this should be implemented separately
		self.camera_on = False
		# camera parameters
		self._dimension = dimension
		self._intrinsics = intrinsics
		self._distortion = distortion

		self._camera_idx = camera
		self.turn_on()

	@property
	def on(self):
		return self.camera_on

	@property
	def dimension(self):
		"""
		Returns the dimension of camera
		"""
		return self._dimension

	@dimension.setter
	def dimension(self, dim):
		self._dimension = dim

	@property
	def intrinsics(self):
		return self._intrinsics

	@intrinsics.setter
	def intrinsics(self, K):
		self._intrinsics = K

	@property
	def distortion(self):
		return self._distortion

	@distortion.setter
	def distortion(self, D):
		self._distortion = D

	def turn_on(self):
		"""
		Turn on the camera;
		Returns an instance of corresponding camera object
		"""
		self.camera_on = True

	def turn_off(self):
		"""
		Turn off the camera;
		"""
		self.camera_on = False

	def stream(self):
		"""
		Only used for ROS subscriber related cameras.
		"""
		try:
			rospy.spin()
		except KeyboardInterrupt:
			self.turn_off()

	def snapshot(self):
		"""
		Return one captured image at the moment it gets called
		"""
		raise NotImplementedError('Each camera class must implement this method individually.')

	def callback(self, img_data, info):
		"""
		A callback function to read image and capture edge 
		corner points from camera
		"""
		foundPattern, points = cv2.findChessboardCorners(
			img_data, info['board_size'], None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH
		)
		cv2.drawChessboardCorners(img_data, info['board_size'], 
			points, foundPattern)
		cv2.imshow('calibrate', img_data)
		key = cv2.waitKey(1) & 0xff

		if key == 115:
			print('Recognizing pattern...')
		else:
			return

		cv2.cornerSubPix(cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY), 
			points, (11, 11), (-1, -1), 
			(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

		if not foundPattern:
			print('Camera did not find pattern...'
				' Please re-adjust checkerboard position to continue.')
				
			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), img_data)
		
		else:
			return points.reshape((info['num_of_points'], 2))


class UVCCamera(Camera):


	def __init__(self, camera_index, dimension=None, 
		intrinsics=None, distortion=None):

		super(UVCCamera, self).__init__(camera_index, 
			dimension, intrinsics, distortion)

	def turn_on(self):
		self._device = cv2.VideoCapture(self._camera_idx)

		# 3, 4 are cv2 constants for width, height
		if not self._dimension:
			self.dimension = (int(self._device.get(3)), int(self._device.get(4)))

		self.camera_on = self._device.isOpened()

	def turn_off(self):

		self._device.release()
		self.camera_on = False

	def snapshot(self):
		if self.camera_on:
			s, img = self._device.read()
			if s:
				return img


class PrimeSense(Camera):

	def __init__(self, camera_index, dimension=None, 
		intrinsics=None, distortion=None):

		super(PrimeSense, self).__init__(camera_index, 
			dimension, intrinsics, distortion)

	# TODO


class Kinect(Camera):

	def __init__(self, camera, dimension=None, 
		intrinsics=None, distortion=None):

		if rospy.get_name() == '/unnamed':
			rospy.init_node('kinect_calibration')
		super(Kinect, self).__init__(camera, 
			dimension, intrinsics, distortion)


	def snapshot(self, callback, info):
		self._device = rospy.Subscriber('/kinect2/{}/image_color_rect'.format(self._camera_idx), 
			Image, callback, callback_args=info)
		
	def turn_off(self):
		cv2.destroyWindow('kinect_calibrate')
		self._device.unregister()
		self.camera_on = False

	def callback(self, img_data, info):

		cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
		found, points = cv2.findChessboardCorners(
			cv_image, info['board_size'], None
		)
		cv2.drawChessboardCorners(cv_image, info['board_size'], 
			points, found)

		cv2.imshow('kinect_calibrate', cv_image)
		key = cv2.waitKey(1) & 0xff

		if key == 115:
			print('Recognizing pattern...')
		else:
			return

		if not found:
			print('Robot camera did not find pattern...'
				' Please re-adjust checkerboard position to continue.')
			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), cv_image)
			return

		elif len(info['point_list']) < info['calibration_points']:

			cv2.cornerSubPix(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY), 
				points, (11, 11), (-1, -1), 
				(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
			info['point_list'].append(points)
			print('Successfully read one point.'
				' Re-adjust the checkerboard to continue...')
			return
		else:
			self.turn_off()
			print('Done sampling points.')


class RobotCamera(Camera):

	def __init__(self, camera_type, dimension=None, 
		intrinsics=None, distortion=None):

		if rospy.get_name() == '/unnamed':
			rospy.init_node('robot_camera_calibration')
		super(RobotCamera, self).__init__(camera_type, 
			dimension, intrinsics, distortion)

	# Cannot implement turn_on and 
	def turn_on(self):
		robot_camera = intera_interface.Cameras()
		if not robot_camera.verify_camera_exists(self._camera_idx):
			rospy.logerr('Invalid camera name: {}, '
				'exit the program.'.format(self._camera_idx))
		
		if self._camera_idx == 'head_camera':
			lnk_name = 'head_pan/' + self._camera_idx
		elif self._camera_idx == 'right_hand_camera':
			lnk_name = 'right_j5/' + self._camera_idx
		else:
			lnk_name = self._camera_idx
		cfg = rosparam.get_param('/robot_config/jcb_joint_config/{}/intrinsics'.format(lnk_name))

		if not self._dimension:
			self.dimension = (int(cfg[0]), int(cfg[1]))

		if not self._intrinsics:
			self.intrinsics = np.array([
				[cfg[2], 	  0, cfg[5]],
				[	  0, cfg[3], cfg[6]],
				[	  0,	  0, 	  1]
				], dtype=np.float32)

		if not self._distortion:
			if self._camera_idx == 'head_camera':
				self.distortion = np.array(cfg[-8:], dtype=np.float32)
			else:
				self.distortion = np.array(cfg[-5:], dtype=np.float32)

		self.camera_on = True
		return robot_camera

	def turn_off(self):
		rospy.loginfo('Shutting down robot camera corner detection')
		cv2.destroyWindow('robot_calibrate')
		self._device.unregister()
		self.camera_on = False

	def snapshot(self, callback, info):
		if self._camera_idx == 'head_camera':
			image_string = 'image_rect_color'	
		else:
			image_string = 'image_rect'
		config = '/'.join(['/io/internal_camera', self._camera_idx,
			image_string])

		self._device = rospy.Subscriber(config, Image, 
			callback, 
			callback_args=info)

	def callback(self, img_data, info):
		
		cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
			
		robotFoundPattern, robot_points = cv2.findChessboardCorners(
			cv_image, info['board_size'], None
		)

		cv2.drawChessboardCorners(cv_image, info['board_size'], 
			robot_points, robotFoundPattern)

		cv2.imshow('robot_calibrate', cv_image)
		key = cv2.waitKey(1) & 0xff

		if key == 115:
			print('Recognizing pattern...')
		else:
			return

		if not robotFoundPattern:
			print('Camera did not recognize pattern...'
				' Please readjust the board')
			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), cv_image)
			return

		elif len(info['point_list']) < info['calibration_points']:

			cv2.cornerSubPix(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY), 
				robot_points, (11, 11), (-1, -1), 
				(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
			info['point_list'].append(np.reshape(
				robot_points, (info['num_of_points'], 2)))

			print('Successfully read one point.'
				' Re-adjust the checkerboard '
				'and press Enter to Continue...')
			return 
		else:
			print('Done sampling points.')
			self.turn_off()


class StereoCamera(Camera):
	"""
	A base class for multiple cameras 
	can be any combination between uvc, primesense, or kinect
	"""
	def __init__(self, left, right, 
		left_dimension, 
		left_intrinsics, 
		left_distortion, 
		right_dimension,
		right_intrinsics,
		right_distortion):

		self.left_on, self.right_on = False, False

		self._left_camera, self._right_camera = left, right

		self._left_dimension = left_dimension
		self._left_intrinsics = left_intrinsics 
		self._left_distortion = left_distortion
		self._right_dimension = right_dimension
		self._right_intrinsics = right_intrinsics
		self._right_distortion = right_distortion

		self.turn_on()

	def turn_on(self):
		"""
		Turn on takes left and right inputs. Snapshot and 
		turn_off inherit from the Camera class
		"""
		if not self._left_camera.on:
			self._left_camera.turn_on()
			self._right_camera.turn_on()
		
		self.left_on = self._left_camera.on
		self.right_on = self._right_camera.on
		self._left_dimension = self._left_camera.dimension
		self._right_dimension = self._right_camera.dimension

	def snapshot(self):
		return self._left_camera.snapshot(), self._right_camera.snapshot()
		
	def turn_off(self):
		self._left_camera.turn_off()
		self._right_camera.turn_off()
		self.left_on = self._left_camera.on
		self.right_on = self._right_camera.on

	@property
	def on(self):
		return self.left_on, self.right_on

	@property
	def dimension(self):
		"""
		Returns the dimension of camera
		"""
		return self._left_dimension, self._right_dimension

	@dimension.setter
	def dimension(self, l_dim, r_dim):
		self._left_dimension = l_dim
		self._right_dimension = r_dim

	@property
	def intrinsics(self):
		return self._left_camera.intrinsics, self._right_camera.intrinsics

	@intrinsics.setter
	def intrinsics(self, lK, rK):
		self._left_camera.intrinsics = lK
		self._right_camera.intrinsics = rK

	@property
	def distortion(self):
		return self._left_camera.distortion, self._right_camera.distortion

	@distortion.setter
	def distortion(self, lD, rD):
		self._left_camera.distortion = lD
		self._right_camera.distortion = rD

	def callback(self, left_img, right_img, info):
		"""
		A callback function to read both images and capture edge 
		corner points, displaying views at the same time
		Returns found patterns for left, right cameras
		"""
		left_found, left_points = cv2.findChessboardCorners(
			left_img, info['board_size'], None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH
		)
		right_found, right_points = cv2.findChessboardCorners(
			right_img, info['board_size'], None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH
		)

		cv2.drawChessboardCorners(left_img, info['board_size'], 
			left_points, left_found)
		cv2.drawChessboardCorners(right_img, info['board_size'], 
			right_points, right_found)

		cv2.imshow('calibrate_left', left_img)
		cv2.imshow('calibrate_right', right_img)
		key = cv2.waitKey(1) & 0xff

		if key == 115:
			print('Recognizing pattern...')
		else:
			return

		cv2.cornerSubPix(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY), 
			left_points, (11, 11), (-1, -1), 
			(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

		cv2.cornerSubPix(cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY), 
			right_points, (11, 11), (-1, -1), 
			(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

		if not left_found or not right_found:
			print('Camera did not find pattern...'
				' Please re-adjust checkerboard position to continue.')
			
			fail_img = left_img if not robotFoundPattern else right_img
			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), fail_img)
		
		else:
			return (left_points.reshape((info['num_of_points'], 2)),
				right_points.reshape((info['num_of_points'], 2)))


class UVCRobotStereo(StereoCamera):
	"""
	Hybrid indicates the two different types of cameras.
	Usually Kinect & robot camera
	Note: left camera is usually the external camera, be 
	UVC or PrimeSense, or Kinect, and the right camera
	is the name string of robot camera, be 
	'head_camera' or 'right_hand_camera' for Sawyer
	"""
	def __init__(self, left, right, 
		left_dimension=None, 
		left_intrinsics=None, 
		left_distortion=None, 
		right_dimension=None,
		right_intrinsics=None,
		right_distortion=None):
		
		super(UVCRobotStereo, self).__init__(left, right, 
			left_dimension, 
			left_intrinsics, 
			left_distortion, 
			right_dimension,
			right_intrinsics,
			right_distortion)

	def snapshot(self, info):

		self._right_camera.snapshot(self.callback, info)
		self.stream()

	def callback(self, img_data, info):
		"""
		Due to the constraint of sampling the same set of points,
		has to read the same set of points in ROS initiated 
		camera callback for the external camera
		"""
		try:
			# Using internal to indicate robot camera (right)
			# and external to indicate UVC/Kinect camera (left)
			internal_img = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')

			# Directly using the left camera
			external_img = self._left_camera.snapshot()

			internal_found, internal_points = cv2.findChessboardCorners(
				internal_img, info['board_size'], None
			)
			external_found, external_points = cv2.findChessboardCorners(
				external_img, info['board_size'], None, 
				cv2.CALIB_CB_ADAPTIVE_THRESH
			)

			cv2.drawChessboardCorners(external_img, info['board_size'], 
				external_points, external_found)
			cv2.drawChessboardCorners(internal_img, info['board_size'], 
				internal_points, internal_found)

			cv2.imshow("internal-calibrate", internal_img)
			cv2.imshow("external-calibrate", external_img)
			
			key = cv2.waitKey(1) & 0xff
			if key == 115:
				print('Recognizing pattern...')
			else:
				return
				
			if not internal_found or not external_found:
				print('At least camera did not find pattern...'
					' Please re-adjust checkerboard position to continue.')
				fail_img = internal_img if not internal_found else external_img
				cv2.imwrite(pjoin(info['directory'], 
					'failures/{}.jpg'.format(rospy.Time.now())), fail_img)

			elif len(info['left_point_list']) < info['calibration_points']:

				cv2.cornerSubPix(cv2.cvtColor(internal_img, cv2.COLOR_BGR2GRAY), 
					internal_points, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
				cv2.cornerSubPix(cv2.cvtColor(external_img, cv2.COLOR_BGR2GRAY), 
					external_points, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

				info['right_point_list'].append(internal_points)
				info['left_point_list'].append(external_points)

				print('Successfully read one point.'
					' Re-adjust the checkerboard to continue...')
			else:
				print('Done sampling points.')
				self.turn_off()

		except CvBridgeError, err:
			rospy.logerr(err)
			return


class KinectRobotStereo(StereoCamera):
	"""
	Hybrid indicates the two different types of cameras.
	Usually Kinect & robot camera
	Note: left camera is usually the external camera, be 
	UVC or PrimeSense, or Kinect, and the right camera
	is the name string of robot camera, be 
	'head_camera' or 'right_hand_camera' for Sawyer
	"""
	def __init__(self, left, right, 
		left_dimension=None, 
		left_intrinsics=None, 
		left_distortion=None, 
		right_dimension=None,
		right_intrinsics=None,
		right_distortion=None):
		
		super(KinectRobotStereo, self).__init__(left, right, 
			left_dimension, 
			left_intrinsics, 
			left_distortion, 
			right_dimension,
			right_intrinsics,
			right_distortion)

	def snapshot(self, info):

		self._left_camera.snapshot(self.kinect_callback, info)
		self._right_camera.snapshot(self.robot_callback, info)

	def kinect_callback(self, img_data, info):
		"""
		Need to run 
		rosrun kinect2_bridge kinect2_bridge _fps_limit:=3
		with limit!
		"""
		cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
		found, points = cv2.findChessboardCorners(
			cv_image, info['board_size'], None
		)

		cv2.drawChessboardCorners(cv_image, info['board_size'], 
			points, found)
		cv2.imshow('kinect_calibrate', cv_image)

		key = cv2.waitKey(1) & 0xff

		# Use space for kinect
		if key == 32:
			print('Kinect recognizing pattern...')
		else:
			return

		if not found:
			print('Kinect did not find pattern...'
				' Please re-adjust checkerboard position to continue.')
			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), cv_image)

		elif len(info['left_point_list']) < info['calibration_points']:

			cv2.cornerSubPix(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY), 
				points, (11, 11), (-1, -1), 
				(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
			info['left_point_list'].append(points)
			print('Kinect successfully read one point.')
		else:
			print('Done sampling points from Kinect.')
			self._left_camera.turn_off()

		print(len(info['left_point_list']), len(info['right_point_list']), 'kinect')

	def robot_callback(self, img_data, info):
		"""
		Due to the constraint of sampling the same set of points,
		has to read the same set of points in ROS initiated 
		camera callback for the external camera
		"""

		# Using internal to indicate robot camera (right)
		internal_img = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
		internal_found, internal_points = cv2.findChessboardCorners(
			internal_img, info['board_size'], None
		)
		
		cv2.drawChessboardCorners(internal_img, info['board_size'], 
			internal_points, internal_found)
		cv2.imshow("robot_calibrate", internal_img)
		
		key = cv2.waitKey(1) & 0xff

		# Use 's' for robot
		if key == 115:
			print('Robot recognizing pattern...')
		else:
			return
			
		if not internal_found:
			print('Robot camera did not find pattern...'
				' Please re-adjust checkerboard position to continue.')

			cv2.imwrite(pjoin(info['directory'], 
				'failures/{}.jpg'.format(rospy.Time.now())), internal_img)

		elif len(info['right_point_list']) < info['calibration_points']:

			cv2.cornerSubPix(cv2.cvtColor(internal_img, cv2.COLOR_BGR2GRAY), 
				internal_points, (11, 11), (-1, -1), 
				(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
			info['right_point_list'].append(internal_points)
			print('Robot successfully read one point.')
		else:
			print('Done sampling points from robot camera.')
			self._right_camera.turn_off()

		print(len(info['left_point_list']), len(info['right_point_list']), 'robot')










