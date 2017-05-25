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
	def __init__(self, boardSize, checkerSize, 
		direct, calib_min, camera_dim):

		self._board_size = boardSize
		self._checker_size = checkerSize

		self._calib_directory = direct
		self._calibration_points = calib_min

		self._numCornerPoints = self._board_size[0] * self._board_size[1]
		self._camera_dim = camera_dim

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
			object_points, self.img_points, self._camera_dim)

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
			(self._numCornerPoints, 3)).astype(np.float32)

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


class RGBDCalibrator(CameraCalibrator):

	def __init__(self, boardSize, checkerSize, direct, 
		camera_dim, camera_idx=0, calib_min=4):

		super(RGBDCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		# Initialize the camera
		# self._camera = cv2.VideoCapture(camera_idx)

	#TODO


class UVCCalibrator(CameraCalibrator):

	def __init__(self, boardSize, checkerSize, direct, 
		camera_dim, camera_idx=0, calib_min=4):

		super(UVCCamCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		# Initialize the camera
		self._camera = cv2.VideoCapture(camera_idx)

	def _get_image_points(self):

		s = False
		while not s:
			s, img = self._camera.read()

		# Sample enough image points
		while len(self.img_points) < self._calibration_points:
			self.camera_callback(img, rospy.Time.now())

	def camera_callback(self, img_data, time_stamp):

		foundPattern, points = cv2.findChessboardCorners(
			img, self._board_size, None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH
		)

		if not foundPattern:
			raw_input('UVC camera did not find pattern...'
				' Please re-adjust checkerboard position' 
				'and press Enter.')
			cv2.imwrite(pjoin(self._calib_directory, 
				'failures/{}.jpg'.format(time_stamp)), img)
		else:
			self.img_points.append(points.reshape((
				self._numCornerPoints, 2)))
			cv2.imwrite(pjoin(self._calib_directory, 
				'uvc/{}.jpg'.format(time_stamp)), img)

			raw_input('Successfully read one point.'
				' Re-adjust the checkerboard and'
				' press Enter to Continue...')


class RobotCalibrator(CameraCalibrator):
	"""
	camera: 'right_hand_camera' and 'head_camera' for sawyer
	"""
	def __init__(self, camera, boardSize, checkerSize, 
		direct, camera_dim, calib_min=4):

		super(RobotCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		rospy.init_node('calibration')
		self._robot_camera = intera_interface.Cameras()
		self._camera_type = camera

		if camera == "head_camera":
			param = rosparam.get_param('/robot_config/'
				'jcb_joint_config/head_pan/' + camera)['intrinsics']
		else:
			param = rosparam.get_param('/robot_config/'
				'jcb_joint_config/right_j5/' + camera)['intrinsics']

		self._internal_intrinsic = np.array([
			[param[2], 0, param[5]], 
			[0, param[3], param[6]], 
			[0,                    0,                   1]], dtype=np.float32)

		if camera == "head_camera":
			self._internal_distortion = np.array(param[-8:], dtype=np.float32)
		else:
			self._internal_distortion = np.array(param[-5:], dtype=np.float32)

	def camera_callback(self, img_data, camera):
		try:
			time_stamp = rospy.Time.now()
			cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
			
			robotFoundPattern, robot_points = cv2.findChessboardCorners(
				cv_image, self._board_size, None
			)

			if not robotFoundPattern:
				raw_input('Robot camera did not find pattern...'
					' Please re-adjust checkerboard position and press Enter.')
				cv2.imwrite(pjoin(self._calib_directory, 
					'failures/{}.jpg'.format(time_stamp)), cv_image)
				return
			elif len(self.image_points) < self._calibration_points:


				cv2.cornerSubPix(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY), 
					robot_points, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

				self.image_points.append(np.reshape(
					robot_points, (self._numCornerPoints, 2)))

				cv2.imwrite(pjoin(self._calib_directory, 
					'{}/{}.jpg'.format(camera, time_stamp), cv_image))

				raw_input('Successfully read one point.'
					' Re-adjust the checkerboard '
					'and press Enter to Continue...')
				return
			else:
				raw_input('Done sampling points. Please Ctrl+C to continue.')

		except CvBridgeError, err:
			rospy.logerr(err)
			return

	def _get_image_points(self):
		"""
		Use a callback function to process captured images
		"""
		if not self._robot_camera.verify_camera_exists(self._camera_type):
			rospy.logerr('Invalid camera name: {}, '
				'exit the program.'.format(self._camera_type))
		
		self._robot_camera.start_streaming(self._camera_type)
		self._robot_camera.set_callback(self._camera_type, 
			self.camera_callback,
			rectify_image=True, 
			callback_args=self._camera_type)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo('Shutting down robot camera corner detection')
			self._robot_camera.stop_streaming(self._camera_type)


class StereoCalibrator(CameraCalibrator):

	"""
	Stereo camera calibration. Requires the two cameras be 
	aligned on the same x-axis. 
	Both cameras are not robot cameras, and camera indices
	must be provided.
	Also assumes two cameras are identical; only requires
	one camera dimension input
	"""
	def __init__(self, camera1, camera2, boardSize, 
		checkerSize, direct, camera_dim, calib_min=4):
		"""
		Note constructor here takes the index of 
		both left and right cameras
		"""
		super(StereoCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		# WARNING: 
		# If two cameras are connected to the same usb card,
		# then it's possible one camera reads None image due 
		# to bandwidth limit. In that case decrease the 
		# resolution by doing:
		# self._left_camera.set(3, 160)
		# self._left_camera.set(4, 120)
		# self._right_camera.set(3, 160)
		# self._right_camera.set(4, 120)

		self._left_camera = cv2.VideoCapture(camera1) 
		self._right_camera = cv2.VideoCapture(camera2)

		self.left_img_points = []
		self.right_img_points = [] # image_points

	def _calibrate_camera(self):

		object_points = self._get_object_points()
		self._get_image_points()

		#TODO: Use cornerSubPix
		retVal, k1, d1, k2, d2, R, T, E, F = cv2.stereoCalibrate(
			object_points, 
			self.left_img_points, 
			self.right_img_points, 

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

		s = False
		# Confirm both cameras can see image
		while not s:
			s1, img1 = self._left_camera.read()
			s2, img2 = self._right_camera.read()
			s = s1 and s2

		# Sample enough image points
		while len(self.left_img_points) < self._calibration_points:
			self.camera_callback(img1, img2, rospy.Time.now())

	def camera_callback(self, left_img, right_img, time_stamp):

		leftFoundPattern, left_points = cv2.findChessboardCorners(
			left_img, self._board_size, None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH)

		rightFoundPattern, right_points = cv2.findChessboardCorners(
			right_img, self._board_size, None, 
			cv2.CALIB_CB_ADAPTIVE_THRESH)

		if not (leftFoundPattern and rightFoundPattern):
			raw_input('At least one camera did not find pattern...'
				' Please re-adjust checkerboard position' 
				'and press Enter.')
		else:
			self.left_img_points.append(left_points.reshape((
				self._numCornerPoints, 2)))
			self.right_img_points.append(right_points.reshape((
				self._numCornerPoints, 2)))

			cv2.imwrite(pjoin(self._calib_directory, 
				'stereo_left/{}.jpg'.format(time_stamp)), left_img)
			cv2.imwrite(pjoin(self._calib_directory, 
				'stereo_right/{}.jpg'.format(time_stamp)), right_img)

			raw_input('Successfully read one point.'
				' Re-adjust the checkerboard and'
				' press Enter to Continue...')


class DuoCalibrator(RobotCalibrator):
	"""
	Duo camera calibration (generalization of stereo)
	By default, camera1 (left) is external camera (usually providing
	clear top view, can be RGBD or UVC); camera 2 is 
	the robot camera used for transforming camera coords 
	to the base coords (for gripper)

	Performs calibration separately, then compute the 
	relation between two cameras
	"""

	def __init__(self, camera1, camera2, boardSize, 
		checkerSize, direct, camera_dim1, camera_dim2, 
		calib_min=4):
		"""
		Note the constructor takes the index of external 
		camera, and the string name of the robot camera (internal)
		"""
		super(DuoCalibrator, self).__init__(camera2, boardSize, 
			checkerSize, direct, camera_dim2, calib_min)

		self._external_camera = cv2.VideoCapture(camera1)

		self._external_camera_dim = camera_dim1
		self._internal_camera_dim = camera_dim2

		self.external_img_points = []

		self.tl = tf.TransformListener()


		self.br = tf.TransformBroadcaster()


		self._external_intrinsic = _UK
		self._external_distortion = _UD

	
	def _calibrate_camera(self):
		"""
		The standard calibration procedure using opencv2
		"""
		# Get sets of points
		object_points = self._get_object_points()
		
		self._get_image_points()

		objp = object_points[0]

		external_to_internal = np.zeros((4, 4), dtype=np.float32)


		retval, k1, d1, r1, t1 = cv2.calibrateCamera(
			object_points, 
			self.external_img_points,
			self._external_camera_dim
			)

		retval, k2, d2, r2, t2 = cv2.calibrateCamera(
			object_points,
			self.image_points,
			self._internal_camera_dim
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

		if self.tl.frameExists(self._camera_type) and self.tl.frameExists('base'):

			t = self.tl.getLatestCommonTime(self._camera_type, 'base')
			hdr = Header(stamp=t, frame_id=self._camera_type)
			translation, rotation = self.tl.lookupTransform('base', self._camera_type, t)

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

	def camera_callback(self, img_data, camera):
		"""
		Due to the constraint of sampling the same set of points,
		has to read the same set of points in ROS initiated 
		camera callback for the external camera
		"""
		try:

			time_stamp = rospy.Time.now()
			
			internal_img = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')
			external_img = self._external_camera.read()[1]

			robotFoundPattern, internal_points = cv2.findChessboardCorners(
				internal_img, self._board_size, None
			)
			#TODO: unsubscribe by itself
			externFoundPattern, external_points = cv2.findChessboardCorners(
				external_img, self._board_size, None, 
				cv2.CALIB_CB_ADAPTIVE_THRESH
			)

			cv2.imshow("internal-calibrate", internal_img)
			cv2.imshow("external-calibrate", external_img)
			
			k = cv2.waitKey(1) & 0xff

			if k == 115:
				print('Recognizing pattern...')
			else:
				return
				
			if not (robotFoundPattern and externFoundPattern):
				print('At least camera did not find pattern...'
					' Please re-adjust checkerboard position to continue.')
				
				fail_img = internal_img if not robotFoundPattern else external_img
				cv2.imwrite(pjoin(self._calib_directory, 
					'failures/{}.jpg'.format(time_stamp)), fail_img)
				return

			elif len(self.image_points) < self._calibration_points:

				cv2.cornerSubPix(cv2.cvtColor(internal_img, cv2.COLOR_BGR2GRAY), 
					internal_points, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

				cv2.cornerSubPix(cv2.cvtColor(external_img, cv2.COLOR_BGR2GRAY), 
					external_points, (11, 11), (-1, -1), 
					(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

				self.image_points.append(internal_points)
				self.external_img_points.append(external_points)

				cv2.imwrite(pjoin(self._calib_directory, 
					'camera/{}.jpg'.format(time_stamp)), external_img)
				cv2.imwrite(pjoin(self._calib_directory,
					'{}/{}.jpg'.format(camera, time_stamp)), internal_img)

				print('Successfully read one point.'
					' Re-adjust the checkerboard to continue...')
			
				return

			else:
				raw_input('Done sampling points. Please Ctrl+C to continue.')

		except CvBridgeError, err:
			rospy.logerr(err)
			return



