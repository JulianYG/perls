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
LENGTH = 0.143


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

        if not isinstance(depth, np.ndarray):
            depth_map = cv2.flip(depth.asarray(np.float32)[1: -1, :], 1)

        else:
            depth_map = depth

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

    def convert_inverse(self, x, y, z):
        """
        Convert boundign box in XYZ space to crop of depth image
        :param x: x
        :param y: y
        :param z: z
        :return: u,v - image pixels coords
        """
        u = x/z * self._intrinsics_RGB[0, 0] +  self._intrinsics_RGB[0, 2]
        v = y/z * self._intrinsics_RGB[1, 1] +  self._intrinsics_RGB[1, 2]
        return u,v

    def crop_by_bb(self, location, size, depth):
        """
        Convert boundign box in XYZ space to crop of depth image
        :param location: x,y,z of upper right back corder of boundign box
        :param size: width,length,height size of bounding box
        :param depth: depth image
        :return: crop - the cropped depth image
        """
        u1,v1 = convert_inverse(location[0], location[1], location[2])
        u2,v2 = convert_inverse(location[0]+depth[0], location[1]+depth[1], location[2]+depth[2])
        return depth[v1:v2,u1:u2]
