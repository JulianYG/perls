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

rospy.init_node('ha')

def rgb_callback(img_data):
    """
    Grasp objects based on mouse click position
    on HD RGB image.
    :return: None
    """

    cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')

    # undistorted_color = cv2.undistort(cv_image, self._intrinsics_RGB, self._distortion_RGB)
    # color = cv2.flip(cv_image, 1)

    cv2.namedWindow('kinect_grasp', cv2.CV_WINDOW_AUTOSIZE)

    
    cv2.imshow('kinect_grasp', cv_image)
      
    cv2.waitKey(1)

    # if self.verbose:
    #     cv2.circle(color, (u, v), 1, (0, 255, 0), 10)
    #     rgb = cv2.resize(color, (1920, 1080))
    #     cv2.imshow("kinect_grasp", rgb)
    #     cv2.waitKey(0)


# rospy.waitForMessage('dsfoji')
_rgb = rospy.Subscriber(
    '/kinect2/hd/image_color_rect',
    Image, rgb_callback)

try:
    rospy.spin()

except KeyboardInterrupt:
    cv2.destroyWindow('kinect_grasp')
  
    sys.exit(0)

