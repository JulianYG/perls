#!/usr/bin/env python
from __future__ import print_function

import sys, os
import rospy
import cv2
import intera_interface
from std_msgs.msg import String
from sensor_msgs.msg import Image
import tf
from cv_bridge import CvBridge, CvBridgeError

from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'ros_'))
from robot import Robot
import time
import numpy as np

limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')
arm = Robot(limb, gripper)

# USB Camera matrix 
_UK = np.array([
    [600.153387, 0, 315.459915], 
    [0, 598.015225, 222.933946], 
    [0,          0,          1]
    ], np.float32)

# USB Camera Distortion
_UD = np.array([0.147084, -0.257330, 
    0.003032, -0.006975, 0.000000], np.float32)

rospy.init_node('grasp', anonymous=True)

class GraspSawyer(object):

    def __init__(self, robot, usbCameraMatrix, 
        usbDistortionVector, boardSize=(9, 6), checkerSize=0.026, 
        usbCameraIndex=0):

        self._robot = robot

        self._usb_intrinsic = usbCameraMatrix
        self._usb_distortion = usbDistortionVector
        self._board_size = boardSize
        self._checker_size = checkerSize
        self._inverse_usb_intrinsic = np.linalg.inv(self._usb_intrinsic)

        # self._init_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
        #     'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

        self._usb_camera = cv2.VideoCapture(usbCameraIndex)
        self._robot_camera = intera_interface.Cameras()
        self.tl = tf.TransformListener()

        self.robot_camera_corner_points = []

        # self._robot.set_init_positions(self._init_pose)
        self._robot.limb.move_to_neutral()

        # Run transform matrix initializer
        robot_camera_corner_points = self._get_robot_camera_points()
        self.pixel_usb_cam_translation, self.pixel_usb_cam_inv_rotation = \
            self._get_camera_transformation(robot_camera_corner_points)

    def _get_robot_camera_points(self):

        corner_points = []
        numCornerPoints = self._board_size[0] * self._board_size[1]

        if not self._robot_camera.verify_camera_exists('head_camera'):
            rospy.logerr("Invalid self._robot_camera name, exiting the example.")

        def sawyer_camera_callback(img_data):

            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
                foundPattern, points = cv2.findChessboardCorners(
                    cv_image, self._board_size, None
                )
                #TODO: unsubscribe by itself
                if foundPattern and not self.robot_camera_corner_points:
                    corner_points.append(points)
                    return
            except CvBridgeError, err:
                rospy.logerr(err)
                return  
        
        self._robot_camera.start_streaming('head_camera')
        self._robot_camera.set_callback('head_camera', sawyer_camera_callback,
            rectify_image=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo('Shutting down robot camera corner detection')
            self._robot_camera.stop_streaming('head_camera')
            
        return np.hstack(
            # Reverse the matrix since two cameras see reverted images
            (np.array(corner_points[0]).reshape((numCornerPoints, 2))[::-1], 
            # Using 1's since pretending in robot camera image frame
            np.ones((numCornerPoints, 1), dtype=np.float32)))

    def _get_camera_transformation(self, robotCamCornerPoints):

        s, foundPattern = False, False
        # Loop until read image
        print('Reading from usb camera...')
        while not s:
            s, img = self._usb_camera.read()

        # Loop until found checkerboard pattern
        print('Finding checkerboard pattern...')
        while not foundPattern:
            foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
                img, boardShape, None, 
                cv2.CALIB_CB_ADAPTIVE_THRESH
            )
        
        retval, rvec, tvec = cv2.solvePnP(
            robotCamCornerPoints, usbCamCornerPoints,
            self._usb_intrinsic, self._usb_distortion)

        rotMatrix = cv2.Rodrigues(rvec)[0]
        invRotation = np.linalg.inv(rotMatrix)

        return tvec, invRotation

    def usb_cam_to_robot_cam(self, u, v):

        # Change the pixel point here (in form [u,v,1])
        tempMat = self.pixel_usb_cam_inv_rotation * self._inverse_usb_intrinsic

        pixelPoint = np.array([u, v, 1]).astype(np.float32) 
        tempMat2 = tempMat.dot(pixelPoint)
        tempMat3 = self.pixel_usb_cam_inv_rotation.dot(
            np.reshape(self.pixel_usb_cam_translation, 3))

        # approx height of each block + 
        # (inv Rotation matrix * inv Camera matrix * point) 
        s = .026 + tempMat3[2]

        # inv Rotation matrix * tvec
        s /= tempMat2[2] 

        # s * [u,v,1] = M(R * [X,Y,Z] - t)  
        # ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
        temp = self._inverse_usb_intrinsic.dot(s * pixelPoint)
        temp2 = temp - np.reshape(translation, 3)
        return self.pixel_usb_cam_inv_rotation.dot(temp2)

    def robot_cam_to_gripper(pixel_location):

        # TODO: figure out how to do the image to camera reference frame in sawyer
        if self.tl.frameExists('/head_camera') and self.tl.frameExists('/right_j6')
            t = self.tl.getLatestCommonTime("/head_camera", "/right_j6")
            translation, rotation_quaternion = self.tf.lookupTransform(
                "/head_camera", "/right_j6", t)
            transform_matrix = self.tl.TransformerROS.fromTranslationRotation(
                translation, rotation_quaternion)
            return transform_matrix * pixel_location.T

    def move_to(self, u, v, z):

        robot_cam_coords = self.usb_cam_to_robot_cam(u, v)
        gripper_coords = self.robot_cam_to_gripper(robot_cam_coords)
        self._robot.reach_absolute({'position': (gripper_coords[0], 
            gripper_coords[1], z), 'orientation': (0, 1, 0, 0)})

    def move_to_with_grasp(self, u, v, hover, dive):
        self.move_to(u, v, hover)
        time.sleep(0.2)
        self.move_to(u, v, dive)
        time.sleep(0.3)
        arm.slide_grasp(0)

    def move_to_with_lift(self, u, v, 
        hover=0.18, dive=0.10, drop=None):

        self.move_to_with_grasp(u, v, hover, dive)
        time.sleep(0.8)
        if drop:
           self.move_to(drop[0], drop[1], hover)
        else:
            self._robot.limb.move_to_neutral()
        time.sleep(0.5)
        arm.slide_grasp(1)

    def grasp_by_click(self):
        
        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                self.move_to_with_lift(x, y)
        try:
            s, img = self._usb_camera.read()
            cv2.namedWindow('grasp-by-click', cv2.CV_WINDOW_AUTOSIZE)
            cv2.setMouseCallback('grasp-by-click', mouse_callback)
            cv2.imshow('grasp-by-click', img)
            cv2.waitKey(2)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

    def grasp_by_color(self, color='blue'):

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
            cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cubePoints.append((x + w / 2., y + h / 2.))

        for cp in cubePoints:
            #TODO: maybe change to grasp
            self.move_to(cp[0], cp[1], 0.15)
            time.sleep(1)


sawyer = GraspSawyer(arm, _UK, _UD)
sawyer.grasp_by_color('yellow')


