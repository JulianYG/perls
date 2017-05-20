#!/usr/bin/env python
from __future__ import print_function

import pickle
import numpy as np

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf
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
    		info_list = self.parse_meta(meta_file)
    		return self._read_params(info_list)

    def _calibrate_camera(self):
    	"""
		The standard calibration procedure using opencv2
    	"""
		# Get sets of points
		object_points = self._get_object_points()
		self._get_image_points()

		# Crank the calibration
		retval, K, d, rvec, tvec = cv2.calibrateCamera(
	        [object_points] * self._calibration_points,
	        self.img_points, self._camera_dim)

	    self._write_params({
	        'head_R': rvec,
	        'head_T': tvec,
	        'head_instrinsic': K, 
	        'head_distortion': d
	        })

	   	self.write_meta()

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

        return object_points

    @staticmethod
    def parse_meta(file):
    	"""
		Static method to parse the metadata file
    	"""
    	raise NotImplementedError

    @staticmethod
   	def write_meta(meta_data):
   		"""
		Outputs a metadata file .yml
   		"""
   		raise NotImplementedError

	def _write_params(self, data):
        """
		Given data, write into files.
		Data: a dictionary with (name, value) as key, value pairs
        """
        for name, mat in data.items():
            with open('{}/{}.p'.format(self._calib_directory, 
            	name), 'wb') as f:
                pickle.dump(mat, f)
        writemeta

    def _read_params(self, names):
    	"""
    	Names: a list of properties in query.
		Reads the data and returns corresponding stored data
		in the same order as given names.
    	"""
        data = {}
        for name in names:
            with open('{}/{}.p'.format(self._calib_directory, 
            	name), 'rb') as f:
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

		super(RobotCamCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		self._robot_camera = intera_interface.Cameras()
		self._camera_type = camera

    

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

		super(StereoCalibrator, self).__init__(boardSize, 
			checkerSize, direct, calib_min, camera_dim)

		self._left_camera = cv2.VideoCapture(camera1) 
		self._right_camera = cv2.VideoCapture(camera2)

		self.left_img_points = []
		self.right_img_points = []

	def _calibrate_camera(self):

		object_points = [self._get_object_points()] * calib_min
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
            'usb_intrinsic': k1,
            'usb_distortion': d1,
            '{}_intrinsic'.format(camera): k2,
            '{}_distortion'.format(camera): d2, 
            'rotation': R,
            'translation': T,
            'essential': E, 
            'fundamental': F
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


class DuoCalibrator(object):
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
		camera_idx=0, calib_min=4):

		if camera1 == 'uvc':
			self._left_camera = UVCCalibrator(
				boardSize, checkerSize, direct, 
				camera_dim1, camera_idx, calib_min)
		elif camera1 == 'rgbd':
			self._left_camera = RGBDCalibrator(
				boardSize, checkerSize, direct, 
				camera_dim1, camera_idx, calib_min)
		else:
			raise NotImplementedError('Unrecognized external camera type')

		self._right_camera = RobotCalibrator(camera, 
			boardSize, checkerSize, direct, 
			camera_dim2, calib_min)

	def _calibrate_camera(self):
    	"""
		The standard calibration procedure using opencv2
    	"""
		# Get sets of points
		object_points = self._get_object_points()
		


		# Crank the calibration
		retval, K, d, rvec, tvec = cv2.calibrateCamera(
	        [object_points] * self._calibration_points,
	        self.img_points, self._camera_dim)

	    self._write_params({
	        'head_R': rvec,
	        'head_T': tvec,
	        'head_instrinsic': K, 
	        'head_distortion': d
	        })

	   	self.write_meta()

	    return K, d, rvec, tvec





