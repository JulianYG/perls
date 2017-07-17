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



import rosparam
rospy.init_node('dsdfs')

br = tf.TransformBroadcaster()

with open(pjoin('../calib_data',  
	'HybridCalibrator_transform.p'), 'rb') as f:
	TR = pickle.load(f)

param = rosparam.get_param('/robot_config/'
		'jcb_joint_config/right_j5/' + 'right_hand_camera')['intrinsics']

internal_intrinsic = np.array([
	[param[2], 0, param[5]], 
	[0, param[3], param[6]], 
	[0,                    0,                   1]], dtype=np.float32)

internal_distortion = np.array(param[-5:], dtype=np.float32)

p = np.array([438, 292, 1], dtype=np.float32).T
x = np.linalg.inv(internal_intrinsic).dot(p)

print(np.linalg.inv(TR[:3, :3]).dot(x - TR[:3, 3]))

print(x)
x = [ 0.12308943,  0.09174353,  0.56    , 1    ]
y = np.array([0.7215373067087614, -0.22962536113906679, 0.16429005664644528, 1.])

print(np.linalg.inv(TR).dot(y))
print(TR.dot(x))
print(TR)


pos = TR[:3, 3]

rot = TR[:3, :3]
x = [ 0.12308943,  0.09174353,  1.     ]
print((rot).dot( x - pos  ), 'ha')
# pos_abs = (0.05309283,
# 0.12304577,
# 0.63757806)
# orn_abs = (0, 0, 0, 1)
orn = np.array(tf.transformations.quaternion_from_matrix(TR))
orn = orn / np.sqrt(np.sum(orn ** 2))
print(orn)
#orn_abs = (0.87822085, -0.34770535 , 0.26621342,  0.19224864)
# orn_abs = [ 0.9739193 , -0.03370887 ,-0.22305033 ,-0.02436092]
while not rospy.is_shutdown():
	br.sendTransform(pos, orn, 
					rospy.Time.now(), 'kinect', 'base')
	rospy.Rate(10.).sleep()

