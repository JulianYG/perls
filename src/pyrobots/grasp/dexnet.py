#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper
"""
import h5py
import rospy
import numpy as np
import pickle
from kinect_tracker import KinectTracker
from autolab_core import Point, RigidTransform, YamlConfig
from dexnet.planning.grasp_planner import GQCnn2DGraspPlanner
from os.path import join as pjoin

if rospy.get_name() == '/unnamed':
    rospy.init_node('dexnet_grasp')

rotation_dir = pjoin('./calib_data/kinect',
                        'KinectTracker_rotation.p')
translation_dir = pjoin('./calib_data/kinect', 'KinectTracker_translation.p')
intrinsics_RGB = np.array([[1.0450585754139581e+03, 0., 9.2509741958808945e+02],
                       [0., 1.0460057005089166e+03, 5.3081782987073052e+02],
                       [0., 0., 1.]], dtype=np.float32)

distortion_RGB = np.array([ 1.8025470248423700e-02, -4.0380385825573024e-02,
       -6.1365440651701009e-03, -1.4119705487162354e-03,
       9.5413324012517888e-04 ], dtype=np.float32)


with open(rotation_dir, 'rb') as f:
    r = pickle.load(f)

with open(translation_dir, 'rb') as f:
    t = np.squeeze(pickle.load(f))
print(r)
print(t)
tracker = KinectTracker(r, t, intrinsics_RGB, distortion_RGB)
color,ir,depth = tracker.snapshot()
DEFAULT_CONFIG = '/home/cvgl_ros/deps/dexnet/dex-net/cfg/scripts/gqcnn.yaml'
config = YamlConfig(DEFAULT_CONFIG)
planner = GQCnn2DGraspPlanner(config)
