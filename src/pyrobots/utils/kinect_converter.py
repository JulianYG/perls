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
LENGTH = 0.133

FINGER_OFFSET = 0.066


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
        target_point[2] += LENGTH + 0.015
        if self.verbose:
            print("== desired endeffector pos: {}".format(target_point * 1000))

        return target_point


    # def click_and_grasp(self):
    #     """
    #     Grasp objects based on mouse click position
    #     on HD RGB image.
    #     :return: None
    #     """
    #     global mouseX, mouseY
    #     mouseX = 0
    #     mouseY = 0

    #     def mouse_callback(event, x, y, flags, params):
    #         if event == 1:
    #             mouseX, mouseY = x, y
    #             gripper_pos = self.convert(x, y)

    #             self.move_to_with_lift(*gripper_pos, hover=0.3, drop_height=0.2)

    #     cv2.namedWindow('kinect-rgb', cv2.CV_WINDOW_AUTOSIZE)
    #     cv2.setMouseCallback('kinect-rgb', mouse_callback)

    #     while True:
    #         color, _, _ = self.snapshot()

    #         undistorted_color = cv2.undistort(color.asarray(), self._intrinsics_RGB, self._distortion_RGB)
    #         color = cv2.flip(undistorted_color, 1)
    #         cv2.circle(color, (mouseX, mouseY), 1, (0, 0, 255), 10)

    #         rgb = cv2.resize(color, (1920, 1080))
    #         cv2.imshow("kinect-rgb", rgb)
    #         self._listener.release(self._frames)

    #         key = cv2.waitKey(delay=1)
    #         if key == ord('q'):
    #             break

    # def snapshot(self):
    #     """
    #     Get the snapshots from 3 Kinect cameras
    #     :return: color RGB image, IR image, depth image
    #     """
    #     self._frames = self._listener.waitForNewFrame()

    #     color, ir, depth = \
    #         self._frames[pylibfreenect2.FrameType.Color],\
    #         self._frames[pylibfreenect2.FrameType.Ir], \
    #         self._frames[pylibfreenect2.FrameType.Depth]

    #     self._registration.apply(
    #         color, depth,
    #         self._undistorted,
    #         self._registered,
    #         bigdepth=self._big_depth,
    #         color_depth_map=self._color_depth_map)

    #     return color, ir, depth

    # def move_to(self, x, y, z):
    #     """
    #     Move to given position.
    #     :param x: x coordinate position relative to robot base
    #     :param y: y coordinate position relative to robot base
    #     :param z: z coordinate position relative to robot base
    #     :return: None
    #     """
    #     end_state = dict(position=(x, y, z), orientation=(0, 1, 0, 0))
    #     self.reach_absolute(end_state)
    #     if self.verbose:
    #         print("== move to {} success".format(arm.endpoint_pose()))

    # def move_to_with_grasp(self, x, y, z, hover, dive):
    #     """
    #     Move to given position and grasp
    #     :param x: refer to <move_to::x>
    #     :param y: refer to <move_to::y>
    #     :param z: refer to <move_to::z>
    #     :param hover: the distance above object before grasping, in meters
    #     :param dive: the distance before gripper slows down
    #     for a grasp, in meters
    #     :return: None
    #     """
    #     gripper.set_position(gripper.MAX_POSITION)
    #     arm.set_joint_position_speed(0.1)
    #     self.move_to(x, y, z + hover)
    #     time.sleep(0.7)
    #     self.move_to(x, y, z + dive)
    #     arm.set_joint_position_speed(0.03)
    #     self.move_to(x, y, z - FINGER_OFFSET)
    #     time.sleep(0.8)
    #     gripper.set_position(0)

    # def move_to_with_lift(self,
    #                       x, y, z,
    #                       hover=0.4,
    #                       dive=0.05,
    #                       drop=True,
    #                       drop_height=0.3):
    #     """
    #     Move to given position, grasp the object,
    #     and lift up the object.
    #     :param x: refer to <move_to::x>
    #     :param y: refer to <move_to::y>
    #     :param z: refer to <move_to::z>
    #     :param hover: refer to <move_to_with_grasp::hover>
    #     :param dive: refer to <move_to_with_grasp::dive>
    #     :param drop: boolean whether drop the object after lift
    #     :param drop_height: the height when releasing grasped object
    #     :return: None
    #     """
    #     self.move_to_with_grasp(x, y, z, hover, dive)
    #     time.sleep(.75)
    #     arm.set_joint_position_speed(0.1)
    #     self.move_to(x, y, z + hover)
    #     time.sleep(0.2)

    #     self.move_to(x, y, z + drop_height)
    #     time.sleep(.8)

    #     if drop:
    #         gripper.set_position(gripper.MAX_POSITION)
    #         time.sleep(.5)
