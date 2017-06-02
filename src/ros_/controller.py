#!/usr/bin/env python

import time, os
import sys
import numpy as np

from os.path import join as pjoin

sys.path.append(os.path.abspath(pjoin(os.path.dirname(__file__), '../src/ros_')))

from utils.uvc_tracker import Tracker
from robot import Robot

import cv2
import pickle

class Controller(object):

	"""
	A class for controlling the robot to perform tasks, such as 
	grasping, stacking, etc.
	"""

	def __init__(self, arm, invRotation_dir, translation_dir, 
		intrinsics_dir, camer_idx=0):

		self._robot = arm
		self._usb_camera = cv2.VideoCapture(camer_idx)

		with open(invRotation_dir, 'rb') as f:
			self.invR = pickle.load(f)

		with open(translation_dir, 'rb') as f:
			self.T = pickle.load(f)

		with open(intrinsics_dir, 'rb') as f:
			self.K = pickle.load(f)

	def move_to(self, u, v, z):
        offset_x = 0.00
        offset_y = -0.00
        gripper_coords = Tracker.convert(
        	u, v, z, 
        	self.K, self.T, self.invR
        	)
        print(gripper_coords, 'gripper coords')
        self._robot.reach_absolute({'position': (gripper_coords[0]+offset_x, 
            gripper_coords[1]+offset_y, z), 'orientation': (0.707, 0.707, 0, 0)})

    def move_to_with_grasp(self, u, v, hover, dive):        
        self.move_to(u, v, hover)
        time.sleep(0.7)
        self.move_to(u, v, dive + 0.05)
        self._robot.limb.set_joint_position_speed(0.03)
        self.move_to(u, v, dive)
        time.sleep(0.8)
        arm.slide_grasp(0)

    def move_to_with_lift(self, u, v, 
        hover=0.4, dive=0.095, drop=None, drop_depth=0.23):

        self.move_to_with_grasp(u, v, hover, dive)
        time.sleep(.75)

        self.move_to(u, v, 0.2)
        time.sleep(0.2)
        self._robot.limb.set_joint_position_speed(0.1)
        if drop:
            self.move_to(u, v, 0.45)
            time.sleep(.75)
            self.move_to(drop[0], drop[1], hover)
            time.sleep(.8)
            self._robot.limb.set_joint_position_speed(0.03)
            self.move_to(drop[0], drop[1], drop_depth)
            time.sleep(.5)
            arm.slide_grasp(1)
            time.sleep(.5)
            self._robot.limb.set_joint_position_speed(0.1)
            self.move_to(drop[0], drop[1], hover)
        else:
            self.move_to(u, v, 0.45)
            time.sleep(.75)
            self.move_to(619, 163, hover)
            time.sleep(.8)
            self.move_to(619, 163, 0.23)
            time.sleep(.5)
            arm.slide_grasp(1)
            time.sleep(.5)
            self.move_to(619, 163, .45)

    def grasp_by_click(self):
        
        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                num = int(raw_input('Enter bin number: \n'))
                if num == 1:
                    drop = (454, 7)
                elif num == 2:
                    drop = (434, 125)
                elif num == 3:
                    drop = (438, 233)
                else:
                    drop = None

                self.move_to_with_lift(x, y, drop=drop)
        while True:
            try:
                s, img = self._usb_camera.read()
                cv2.namedWindow('grasp-by-click', cv2.CV_WINDOW_AUTOSIZE)
                cv2.setMouseCallback('grasp-by-click', mouse_callback)
                # Remove distoration
                rectified_image = cv2.undistort(self._usb_intrinsic, 
                    self._usb_distortion)
                cv2.imshow('grasp-by-click', rectified_image)
                cv2.waitKey(1)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()

    def grasp_by_color(self, color='blue'):

        im = self._usb_camera.read()[1]
        im = cv2.bilateralFilter(im,9,75,75)
        im = cv2.fastNlMeansDenoisingColored(im,None,10,10,7,21)
        hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV) 

        if color == 'blue':
            COLOR_MIN = np.array([110, 50, 50], dtype=np.uint8)
            COLOR_MAX = np.array([130, 255, 255], dtype=np.uint8)

            # Thresholding image
            frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)    

        elif color == 'red':
            COLOR_MIN = np.array([0,50,50], np.uint8)
            COLOR_MAX = np.array([10,255,255], np.uint8)
            mask0 = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
            
            COLOR_MIN = np.array([170,50,50], np.uint8)
            COLOR_MAX = np.array([180,255,255], np.uint8)
            mask1 = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
            
            frame_threshed = mask0 + mask1

        elif color == 'yellow':
            # HSV color code lower and upper bounds
            COLOR_MIN = np.array([20, 100, 100], np.uint8)       
            COLOR_MAX = np.array([30, 255, 255], np.uint8)
            
            frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)

        else:
            raise NotImplementedError('Unrecognized color')

        ret, thresh = cv2.threshold(frame_threshed, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, 
            cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cubePoints = []

        for k, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            # cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cubePoints.append((x + w / 2., y + h / 2.))

        for cp in cubePoints:
            #TODO: maybe change to grasp
            self.move_to_with_lift(cp[0], cp[1])
            time.sleep(1)

   def stack(self):

        origin = []
        cnt = 0

        def mouse_callback(event, x, y, flags, param):

            if event == 1:
                
                if not param[0]:
                    param[0].append((x, y))
                else:
                    self.move_to_with_lift(x, y, drop_depth=param[1] * 0.026 + 0.095, drop=param[0][0])
                    param[1] += 1

                self.move_to(498, 52, 0.4)
                print(param)
        while True:
            try:
                s, img = self._usb_camera.read()
                cv2.namedWindow('grasp-by-click', cv2.CV_WINDOW_AUTOSIZE)
                cv2.setMouseCallback('grasp-by-click', mouse_callback, param=[origin, cnt])
                cv2.imshow('grasp-by-click', img)
                cv2.waitKey(1)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()



rospy.init_node('track')

limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, gripper)

ctrlr = Controller(robot, 
	pjoin(os.getcwd(), '../../tools/calibration/calib_data/Tracker_inverseRotation.p',
		'../../tools/calibration/calib_data/Tracker_translation.p',
		'../../tools/calibration/calib_data/Tracker_intrinsics.p'))

ctrlr.grasp_by_click()




