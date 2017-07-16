#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper 
"""

import pickle
import yaml
import numpy as np
from matplotlib import pyplot as plt

import sys, os
from os.path import join as pjoin

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from pylibfreenect2 import Frame
import rospy
import rosparam
import intera_interface
import time
import pybullet as p
sys.path.append(os.path.abspath(os.path.join(__file__, '../../')))
from robot import Robot

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

from hamming.detect import detect_markers


class KinectTracker():

    def __init__(self, robot,
        board_size=(2,2), itermat=(8, 9),
        calib='../../../tools/calibration/calib_data/kinect'):

        self._board_size = board_size
        self._checker_size = 0.0274


        self._arm = robot

        self._intrinsics = None
        self._grid = itermat

        self._calib_directory = calib

        self._transformation = np.zeros((4, 4), dtype=np.float32)
        self._transformation[3, 3] = 1
        self.turn_on()
        self.track()

    def turn_on(self):

        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

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
        if self._intrinsics is None:

            self._intrinsics = self._to_matrix(self._device.getColorCameraParams())

            self._registration = Registration(self._device.getIrCameraParams(), 
                self._device.getColorCameraParams())
        else:
            # IrParams = IrCameraParams(self.K, ...)
            # colorParams = ColorCameraParams(self.K, ...)

            # registration = Registration(IrParams, colorParams)
            self._registration = Registration(
                            self._device.getIrCameraParams(),
                            self._device.getColorCameraParams())

        self._undistorted = Frame(512, 424, 4)
        self._registered = Frame(512, 424, 4)

        self._big_depth = Frame(1920, 1082, 4)
        self._color_depth_map = np.zeros((424, 512),  np.int32).ravel()

        self.camera_on = True

    def snapshot(self):

        self._frames = self._listener.waitForNewFrame()

        color, ir, depth = \
            self._frames[pylibfreenect2.FrameType.Color],\
            self._frames[pylibfreenect2.FrameType.Ir], \
            self._frames[pylibfreenect2.FrameType.Depth]

        self._registration.apply(color, depth, 
            self._undistorted,
            self._registered, 
            bigdepth=self._big_depth, 
            color_depth_map=self._color_depth_map)

        return color, ir, depth

    def get_point_3d(self, x, y):
        return self._registration.getPointXYZ(Frame(512, 424, 4), x, y)

    def stream(self):

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                print(self.get_point_3d(x, y))

        while True:
            
            color, ir, depth = self.snapshot()

            cv2.namedWindow('kinect-ir', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('kinect-ir', ir.asarray() / 65535.)
            cv2.setMouseCallback('kinect-ir', mouse_callback, ir)

            cv2.namedWindow('kinect-depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("kinect-depth", depth.asarray() / 4500.)
            cv2.setMouseCallback('kinect-depth', mouse_callback, depth)

            cv2.namedWindow('kinect-rgb', cv2.WINDOW_AUTOSIZE)
            rgb = cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3)))
            cv2.imshow("kinect-rgb", rgb)

            cv2.namedWindow('kinect-registered', cv2.WINDOW_AUTOSIZE)
            registered = self._registered.asarray(np.uint8)
            cv2.imshow("kinect-registered", registered)
            cv2.setMouseCallback('kinect-registered', mouse_callback, self._registered)

            cv2.namedWindow('kinect-big_depth', cv2.WINDOW_AUTOSIZE)
            big_depth = cv2.resize(self._big_depth.asarray(np.float32), 
                (int(1920 / 3), int(1082 / 3)))
            cv2.imshow("kinect-big_depth", big_depth)
            cv2.setMouseCallback('kinect-registered', mouse_callback, self._big_depth)

            cv2.namedWindow('kinect-rgbd', cv2.WINDOW_AUTOSIZE)
            rgbd = self._color_depth_map.reshape(424, 512)
            cv2.imshow("kinect-rgbd", rgbd)
            # cv2.setMouseCallback('kinect-rgbd', mouse_callback, self._color_depth_map)

            self._listener.release(self._frames)

            key = cv2.waitKey(delay=1)
            if key == ord('q'):
                break

    def match_eval(self):

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                print(self.convert(x, y))

        while True:
            
            color, _, _ = self.snapshot()

            big_depth = cv2.resize(self._big_depth.asarray(np.float32), 
                (int(1920 / 3), int(1082 / 3)))

            cv2.namedWindow('kinect-registered', cv2.WINDOW_AUTOSIZE)
            registered = self._registered.asarray(np.uint8)
            cv2.imshow("kinect-registered", registered)
            cv2.setMouseCallback('kinect-registered', mouse_callback, self._registered)


            self._listener.release(self._frames)
            
            key = cv2.waitKey(delay=1)
            if key == ord('q'):
                break

    def turn_off(self):
        self._device.stop()
        self._device.close()
        self.camera_on = False

    def track(self):

        invRotation_dir = pjoin(self._calib_directory, 'KinectTracker_rotation.p')
        translation_dir = pjoin(self._calib_directory, 'KinectTracker_translation.p')

        if os.path.exists(invRotation_dir) and os.path.exists(translation_dir):
            
            with open(invRotation_dir, 'rb') as f:
                rotation = pickle.load(f)

            with open(translation_dir, 'rb') as f:
                translation = np.squeeze(pickle.load(f))
        
            self._transformation[:3, :3] = rotation
            self._transformation[:3, 3] = translation
            return

        origin = np.array([0.43489, -0.2240, 0.1941], dtype=np.float32)
        orn = np.array([0, 0, 0, 1], dtype=np.float32)

        calibration_grid = np.zeros((self._grid[0] * self._grid[1], 3), np.float32)
        calibration_grid[:, :2] = np.mgrid[0: self._grid[0], 
                                           0: self._grid[1]].T.reshape(-1, 2) * 0.033
        # And randomness to z                                  
        calibration_grid[:, -1] += np.random.uniform(-0.08, 0.2, 
            self._grid[0] * self._grid[1])

        camera_points, gripper_points = [], []

        for pos in calibration_grid:

            # Set position to reach for
            target = origin + pos

            # Add randomness to orientation
            orn = p.getQuaternionFromEuler(np.random.uniform(
                [-np.pi/12., -np.pi/12., -np.pi/6.],[np.pi/12., np.pi/12., np.pi/6.]))

            end_state = dict(position=tuple(target),
                         orientation=orn)
            # Move to target position
            self._arm.reach_absolute(end_state)

            # Wait till stabilize
            time.sleep(2)

            # Get the real position
            gripper_pos = np.array(self._arm.get_tool_pose()[0], 
                                   dtype=np.float32)
            # Match with pattern location

            detected, pix = self._detect_center()

            # Skip if not found pattern
            if not detected:
                print('Pattern not detected. Skipping')
            else:
                print(pix)
                camera_points.append(pix)
                gripper_points.append(gripper_pos)

        # Solve for matrices
        retval, rvec, translation = cv2.solvePnP(
            np.array(gripper_points, dtype=np.float32), 
            # Only use the x, y to get R | T
            np.array(camera_points, dtype=np.float32)[:, :2], 
            self._intrinsics, 
            # Give none, assuming feeding in undistorted images 
            None)
            # self.d)

        rotation = cv2.Rodrigues(rvec)[0]
            
        data = dict(rotation=rotation,
                    translation=translation,)
                    # intrinsics=self.K,
                    # distortion=self.d)

        for name, mat in data.items():
            with open(pjoin(self._calib_directory, 
                '{}_{}.p'.format(self.__class__.__name__, name)), 'wb') as f:
                pickle.dump(mat, f)

        print(rotation, translation)

        self._transformation[:3, :3] = rotation
        self._transformation[:3, 3] = np.squeeze(translation)

        self.turn_off()

    def convert(self, u, v):

        p = np.ones((4,), dtype=np.float32)

        p[:3] = self._registration.getPointXYZ(Frame(int(1920 / 3), int(1080 / 3), 4), u, v)

        print(p, self._transformation)
        return self._transformation.dot(p)

    def _detect_center(self):

        # Take picture
        self.snapshot()
        self._listener.release(self._frames)

        img = self._registered.asarray(np.uint8)
        # img = cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3)))

        cv2.namedWindow('kinect-calibration', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('kinect-calibration', img)

        markers = detect_markers(img)

        if not markers:
            return False, None

        else:
            pos = markers[0].center
            # markers[0].highlight
            marker.highlite_marker(img)
            if pos is None:
                return False, None
            else:
                return True, pos

        # # Look for pattern
        # foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
        #     img, self._board_size, None, 
        #     cv2.CALIB_CB_ADAPTIVE_THRESH
        # )
        cv2.waitKey(500)
        # if foundPattern:
        #     cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 
        #             usbCamCornerPoints, (11, 11), (-1, -1), 
        #             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        #     usbCamCornerPoints = np.squeeze(usbCamCornerPoints)
        #     recognized_pix = np.mean(usbCamCornerPoints, axis=0)

        #     cv2.drawChessboardCorners(img, self._board_size, 
        #         usbCamCornerPoints, foundPattern)

        #     # frame_pos = self._camera.get_point_3d(recognized_pix)

        #     return True, recognized_pix
        # else:
        #     return False, None

    @staticmethod
    def _to_matrix(param):

        return np.array([
            [param.fx, 0, param.cx], 
            [0, param.fy, param.cy], 
            [0, 0, 1]], 
            dtype=np.float32)


if rospy.get_name() == '/unnamed':
    rospy.init_node('kinect_tracker')


limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, None)

dimension = (1920, 1080)
intrinsics = np.array([[1.0741796970429734e+03, 0., 9.3488214133804252e+02], 
                       [0., 1.0640064260133906e+03, 6.0428649994134821e+02], 
                       [0., 0., 1.]], dtype=np.float32)

distortion = np.array([ 3.4454149657654337e-02, 9.7555269933206498e-02,
       1.2981879029576470e-02, 2.7898744906562916e-04,
       -2.0379307133765162e-01 ], dtype=np.float32)

tracker = KinectTracker(robot, board_size=(9,6), itermat=(3,3))

tracker.match_eval()



