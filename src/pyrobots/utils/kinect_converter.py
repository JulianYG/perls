#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper
"""

import time
import pickle
import numpy as np

import sys, os, cv2
from os.path import join as pjoin
sys.path.append(os.path.abspath('../'))

KINECT_DEPTH_SHIFT = -22.54013555237548
GRIPPER_SHIFT = 0.0251
LENGTH = 0.16


class KinectConverter:


    def __init__(self, k, d, 
        param_dir='../../../tools/calibration/calib_data/kinect',
        verbose=False):

        self.verbose = verbose
        self._intrinsics_RGB = k
        self._distortion_RBG = d

        rotation_dir = pjoin(param_dir, 'KinectTracker_rotation.p')
        translation_dir = pjoin(param_dir, 'KinectTracker_translation.p')

        with open(rotation_dir, 'rb') as f:
            r = pickle.load(f)

        with open(translation_dir, 'rb') as f:
            t = np.squeeze(pickle.load(f))

        self._transformation = np.eye(4)
        self._transformation[:3, :3] = r
        self._transformation[:3, 3] = t

    def convert(self, u, v, depth):
        """
        Convert pixel values to the gripper coordinates
        :param u: pixel x (width)
        :param v: pixle y (height)
        :return: vec3 float cartesian position of end effector
        """
        point = np.ones((4,), dtype=np.float32)
        
        depth_map = cv2.flip(depth.asarray(np.float32)[1:-1, :], 1)
        z = depth_map[v, u] + KINECT_DEPTH_SHIFT

        cam_x = (u - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * z
        cam_y = (v - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * z
        point[:3] = (cam_x, cam_y, z)
        target_point = np.linalg.inv(self._transformation).dot(point)[:3] / 1000
        
        if self.verbose:
            print("== depth: {}".format(depth_avg))
            print("== xyz in robot frame: {}".format(target_point * 1000))

        # Prevent collision
        target_point[2] += LENGTH + 0.02
        if self.verbose:
            print("== desired endeffector pos: {}".format(target_point * 1000))

        return target_point
