#!/usr/bin/env python
from __future__ import print_function

import sys, os
import rospy
import cv2
import intera_interface
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'ros_'))
from robot import Robot

import numpy as np

rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

arm = Robot(limb, gripper)

rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
    'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

arm.set_init_positions(rest_pose)

_INIT_POS = np.array(arm.get_tool_pose()[0][:2])
_PXL = (457, 192)

_ALPHA = 0.026 / 21

def pixel_diff_to_pos(curr_pos, curr_pxl, targ_pxl):
    dist = (np.array(targ_pxl) - np.array(curr_pxl)) * _ALPHA
    return curr_pos + dist[::-1]

def get_current_pixel(curr_pos):
    pxl = ((curr_pos - _INIT_POS)[::-1] / _ALPHA).astype(np.int32) + _PXL
    return (pxl[0], pxl[1])

def move_to_pixel_without_reset(x, y):

    # Ignore z-axis value
    gripper_pos = np.array(arm.get_tool_pose()[0][:2])
    curr_pxl = get_current_pixel(gripper_pos)

    target_pos = pixel_diff_to_pos(gripper_pos, curr_pxl, (x, y))
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        0.12), 'orientation': (0, 1, 0, 0)})

move_to_pixel_without_reset(494, 392)

# arm.reach_absolute({'position': (0.44989269453228237, 0.15755170628471973, 0.2), 'orientation': (0, 1, 0, 0)})
# class image_converter:

#     def __init__(self):
        
#         self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)

#     def callback(self,data):

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         (rows,cols,channels) = cv_image.shape
#         if cols > 60 and rows > 60:
#             cv2.circle(cv_image, (50, 50), 10, 255)

#         cv2.imshow("Image window", cv_image)
#         cv2.waitKey(3)

#         try:

#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

#         except CvBridgeError as e:
#             print(e)


