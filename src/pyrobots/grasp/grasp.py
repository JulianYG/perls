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


import pylibfreenect2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from pylibfreenect2.libfreenect2 import IrCameraParams, ColorCameraParams
try:
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()

from sawyer import SawyerArm
from utils.kinect_converter import KinectConverter

KINECT_DEPTH_SHIFT = -22.54013555237548
GRIPPER_SHIFT = 0.0251
LENGTH = 0.133

FINGER_OFFSET = 0.066


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

        self._robot = robot
        self._converter = converter

        # Setup Kinect library framework
        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        # Turn on Kinect camera
        serial = fn.getDeviceSerialNumber(0)
        self._fn = fn
        self._serial = serial
        self._device = self._fn.openDevice(self._serial, pipeline=pipeline)
        self._listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self._device.setColorFrameListener(self._listener)
        self._device.setIrAndDepthFrameListener(self._listener)
        self._device.start()

        # NOTE: must be called after device.start()
        self._intrinsics_RGB = k
        self._distortion_RGB = d

        self._registration = Registration(self._device.getIrCameraParams(),
            self._device.getColorCameraParams())

        # Setup the frames
        self._undistorted = Frame(512, 424, 4)
        self._registered = Frame(512, 424, 4)

        self._big_depth = Frame(1920, 1082, 4)
        self._color_depth_map = np.zeros((424, 512),  np.int32).ravel()

    @staticmethod
    def clean_shutdown():
        """
        Safely and cleanly shut down the system
        :return: None
        """
        rospy.loginfo('Shutting down kinect calibration.')
        cv2.destroyWindow('kinect_calibrate')

    def grasp_by_click(self):
        """
        Grasp objects based on mouse click position
        on HD RGB image.
        :return: None
        """
        global mouseX, mouseY
        mouseX = 0
        mouseY = 0

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                mouseX, mouseY = x, y
                gripper_pos = self._converter.convert(x, y, self._big_depth)

                self.move_to_with_lift(*gripper_pos, hover=0.3, drop_height=0.2)

        cv2.namedWindow('kinect-rgb', cv2.CV_WINDOW_AUTOSIZE)
        cv2.setMouseCallback('kinect-rgb', mouse_callback)

        while True:
            color, _, _ = self.snapshot()

            undistorted_color = cv2.undistort(color.asarray(), self._intrinsics_RGB, self._distortion_RGB)
            color = cv2.flip(undistorted_color, 1)
            cv2.circle(color, (mouseX, mouseY), 1, (0, 0, 255), 10)

            rgb = cv2.resize(color, (1920, 1080))
            cv2.imshow("kinect-rgb", rgb)
            self._listener.release(self._frames)

            key = cv2.waitKey(delay=1)
            if key == ord('q'):
                break

    def grasp(self, visitor_func):

        color, _, _ = self.snapshot()
        undistorted_color = cv2.undistort(color.asarray(), self._intrinsics_RGB, self._distortion_RGB)
        color = cv2.flip(undistorted_color, 1)

        u, v = visitor_func(color)
        gripper_pos = self._converter.convert(u, v, self._big_depth)

        self._listener.release(self._frames)

        self.move_to_with_lift(*gripper_pos, hover=0.3, drop_height=0.2)

        if self.verbose:
            cv2.circle(color, (u, v), 1, (0, 255, 0), 10)
            rgb = cv2.resize(color, (1920, 1080))
            cv2.imshow("kinect-rgb", rgb)
            cv2.waitKey(0)

    def snapshot(self):
        """
        Get the snapshots from 3 Kinect cameras
        :return: color RGB image, IR image, depth image
        """
        self._frames = self._listener.waitForNewFrame()

        color, ir, depth = \
            self._frames[pylibfreenect2.FrameType.Color],\
            self._frames[pylibfreenect2.FrameType.Ir], \
            self._frames[pylibfreenect2.FrameType.Depth]

        self._registration.apply(
            color, depth,
            self._undistorted,
            self._registered,
            bigdepth=self._big_depth,
            color_depth_map=self._color_depth_map)

        return color, ir, depth

    def move_to(self, x, y, z, orn=(0, 0, 0, 1)):
        """
        Move to given position.
        :param x: x coordinate position relative to robot base
        :param y: y coordinate position relative to robot base
        :param z: z coordinate position relative to robot base
        :return: None
        """
        self._robot.tool_pose = ((x, y, z), orn)
        if self.verbose:
            print("== move to {} success".format(self._robot.tool_pose))

    def move_to_with_grasp(self, x, y, z, hover, dive):
        """
        Move to given position and grasp
        :param x: refer to <move_to::x>
        :param y: refer to <move_to::y>
        :param z: refer to <move_to::z>
        :param hover: the distance above object before grasping, in meters
        :param dive: the distance before gripper slows down
        for a grasp, in meters
        :return: None
        """
        self._robot.grasp(1)
        self._robot.set_max_speed(0.1)
        self.move_to(x, y, z + hover)
        time.sleep(0.7)
        self.move_to(x, y, z + dive)
        self._robot.set_max_speed(0.03)
        self.move_to(x, y, z - FINGER_OFFSET)
        time.sleep(0.8)
        self._robot.grasp(0)

    def move_to_with_lift(self,
                          x, y, z,
                          hover=0.4,
                          dive=0.05,
                          drop=True,
                          drop_height=0.3):
        """
        Move to given position, grasp the object,
        and lift up the object.
        :param x: refer to <move_to::x>
        :param y: refer to <move_to::y>
        :param z: refer to <move_to::z>
        :param hover: refer to <move_to_with_grasp::hover>
        :param dive: refer to <move_to_with_grasp::dive>
        :param drop: boolean whether drop the object after lift
        :param drop_height: the height when releasing grasped object
        :return: None
        """
        self.move_to_with_grasp(x, y, z, hover, dive)
        time.sleep(.75)
        self._robot.set_max_speed(0.1)
        self.move_to(x, y, z + hover)
        time.sleep(0.2)

        self.move_to(x, y, z + drop_height)
        time.sleep(.8)

        if drop:
            self._robot.grasp(1)
            time.sleep(.5)

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
    tracker.grasp_by_click()
