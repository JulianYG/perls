#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper
"""

import pickle
import numpy as np
from matplotlib import pyplot as plt

import sys, os, cv2
from os.path import join as pjoin

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

import rospy
import rosparam
import intera_interface

import time

sys.path.append(os.path.abspath('../'))

from sawyer import SawyerArm
from utils.kinect_converter import KinectConverter


class Tracker:
    """
    The click to grasp interface tracker of Kinect camera
    and sawyer robot.
    """
    def __init__(self, robot, converter, k, d, verbose=False):
        """
        Initialize the Kinect tracker.
        :param r_mat: extrinsic rotation matrix
        :param t_vec: extrinsic translation vector
        :param k: Kinect RGB intrinsic matrix
        :param d: Kinect RGB distortion vector
        :param verbose: boolean of print out extra message
        """
        if rospy.get_name() == '/unnamed':
            rospy.init_node('kinect_calibration')

        self.verbose = verbose

        self.mouseX = 0
        self.mouseY = 0

        self._robot = robot
        self._converter = converter

        self._intrinsics_RGB = k
        self._distortion_RGB = d

        self._rgb = None
        self._big_depth = None

    @staticmethod
    def clean_shutdown():
        """
        Safely and cleanly shut down the system
        :return: None
        """
        rospy.loginfo('Shutting down kinect calibration.')
        cv2.destroyWindow('kinect_grasp')

    def rgb_callback(self, img_data, info):
        """
        Grasp objects based on mouse click position
        on HD RGB image.
        :return: None
        """
        print(info)

        cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')

        # undistorted_color = cv2.undistort(cv_image, self._intrinsics_RGB, self._distortion_RGB)
        color = cv2.flip(cv_image, 1)

        cv2.namedWindow('kinect_grasp', cv2.CV_WINDOW_AUTOSIZE)

        if info[0] == 'mouse':
            cv2.setMouseCallback('kinect_grasp', info[1])
            cv2.imshow('kinect_grasp', color)
        else:
            self.mouseX, self.mouseY = info[1](color)
            
        cv2.waitKey(1)

        # if self.verbose:
        #     cv2.circle(color, (u, v), 1, (0, 255, 0), 10)
        #     rgb = cv2.resize(color, (1920, 1080))
        #     cv2.imshow("kinect_grasp", rgb)
        #     cv2.waitKey(0)

    def depth_callback(self, img_data, info):

        depth_image = CvBridge().imgmsg_to_cv2(img_data, 'mono8')
        if 0 < self.mouseX <= 1920 and 0 < self.mouseY <= 1080:
            gripper_pos = self._converter.convert(
                self.mouseX, self.mouseY, depth_image)
            self._robot.move_to_with_lift(*gripper_pos, hover=0.3, drop_height=0.2)
        else:
            print('x, y coordinates out of pixel range')

    def run(self, info):

        self._rgb = rospy.Subscriber(
            '/kinect2/hd/image_color_rect',
            Image, self.rgb_callback, callback_args=info)

        self._big_depth = rospy.Subscriber(
            '/kinect2/hd/image_depth_rect',
            Image, self.depth_callback)

        try:
            rospy.spin()

        except KeyboardInterrupt:
            cv2.destroyWindow('kinect_grasp')
            self._rgb.unregister()
            self._big_depth.unregister()
            sys.exit(0)

if __name__ == '__main__':
    if rospy.get_name() == '/unnamed':
        rospy.init_node('kinect_grasp')

    intrinsics_RGB = np.array([
        [1.0450585754139581e+03, 0., 9.2509741958808945e+02],
        [0., 1.0460057005089166e+03, 5.3081782987073052e+02],
        [0., 0., 1.]], dtype=np.float32)

    distortion_RGB = np.array([
        1.8025470248423700e-02, -4.0380385825573024e-02,
        -6.1365440651701009e-03, -1.4119705487162354e-03,
        9.5413324012517888e-04], dtype=np.float32)

    converter = KinectConverter(intrinsics_RGB, distortion_RGB)
    sawyer = SawyerArm()
    tracker = Tracker(sawyer, converter, intrinsics_RGB, distortion_RGB)

    def mouse_callback(event, x, y, flags, params):
        if event == 1:
            tracker.mouseX, tracker.mouseY = x, y

    tracker.run(('mouse', mouse_callback))
