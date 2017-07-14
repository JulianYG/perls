import pickle
import yaml
import numpy as np
from matplotlib import pyplot as plt

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

import rospy
import rosparam
import intera_interface
import time

import sys, os

sys.path.append(os.path.abspath('../'))
from robot import Robot
import pybullet as p

rospy.init_node('move')

_grid = (8, 9)
limb = intera_interface.Limb('right')
_arm = Robot(limb, None
    )
origin = np.array([0.3690, -0.2566, 0.27], dtype=np.float32)
orn = np.array([0, 0, 0, 1], dtype=np.float32)

calibration_grid = np.zeros((_grid[0] * _grid[1], 3), np.float32)
calibration_grid[:, :2] = np.mgrid[0: _grid[0], 
                                   0: _grid[1]].T.reshape(-1, 2) * 0.033
# And randomness to z                                  
calibration_grid[:, -1] += np.random.uniform(-0.08, 0.2, 
    _grid[0] * _grid[1])

for pos in calibration_grid:

    # Set position to reach for
    target = origin + pos

    # Add randomness to orientation
    # target_orn = orn + np.random.normal(0, 0.05, 4)
    # target_orn /= np.sqrt(np.sum(target_orn ** 2))
    orn = p.getQuaternionFromEuler(np.random.uniform([-np.pi/12., -np.pi/12., -np.pi/6.],[np.pi/12., np.pi/12., np.pi/6.]))
    end_state = dict(position=tuple(target),
                 orientation=orn)#p.getQuaternionFromEuler((0,0,0,1)))
    # Move to target position
    _arm.reach_absolute(end_state)

    # Wait till stabilize
    raw_input()

    