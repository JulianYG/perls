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




rospy.init_node('dsdfs')

br = tf.TransformBroadcaster()

with open(pjoin('../calib_data',  
	'DuoCalibrator_transform.p'), 'rb') as f:
	TR = pickle.load(f)

print(TR)
pos = TR[:3, 3]

# pos = (0.05309283,
# 0.12304577,
# 0.63757806)
# orn = (0, 0, 0, 1)
orn = np.array(tf.transformations.quaternion_from_matrix(TR))
orn = orn / np.sqrt(np.sum(orn ** 2))
print(orn)
#orn = (0.87822085, -0.34770535 , 0.26621342,  0.19224864)
# orn = [ 0.9739193 , -0.03370887 ,-0.22305033 ,-0.02436092]
while not rospy.is_shutdown():
	br.sendTransform(pos, orn, 
					rospy.Time.now(), 'usb', 'base')
	rospy.Rate(10.).sleep()

