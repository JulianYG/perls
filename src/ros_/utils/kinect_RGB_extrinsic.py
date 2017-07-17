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

class KinectTracker():

    def __init__(self, robot,
        board_size=(2,2), itermat=(8, 9), intrinsics_RGB=None, distortion_RGB=None,
        calib='../../../tools/calibration/calib_data/kinect'):

        self._board_size = board_size
        self._checker_size = 0.0247

        self._arm = robot

        # self._intrinsics = None
        self._grid = itermat

        self._intrinsics_RGB = intrinsics_RGB
        self._distortion_RGB = distortion_RGB

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
        if self._intrinsics_RGB is None:

            self._intrinsics_RGB = self._to_matrix(self._device.getColorCameraParams())

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

    # def stream(self):

    #     def mouse_callback(event, x, y, flags, params):
    #         if event == 1:
    #             print(self.get_point_3d(x, y))

    #     while True:
            
    #         color, ir, depth = self.snapshot()

    #         cv2.namedWindow('kinect-ir', cv2.CV_WINDOW_AUTOSIZE)
    #         cv2.imshow('kinect-ir', ir.asarray() / 65535.)
    #         cv2.setMouseCallback('kinect-ir', mouse_callback, ir)

    #         cv2.namedWindow('kinect-depth', cv2.CV_WINDOW_AUTOSIZE)
    #         cv2.imshow("kinect-depth", depth.asarray() / 4500.)
    #         cv2.setMouseCallback('kinect-depth', mouse_callback, depth)

    #         cv2.namedWindow('kinect-rgb', cv2.CV_WINDOW_AUTOSIZE)
    #         rgb = cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3)))
    #         cv2.imshow("kinect-rgb", rgb)

    #         cv2.namedWindow('kinect-registered', cv2.CV_WINDOW_AUTOSIZE)
    #         registered = self._registered.asarray(np.uint8)
    #         cv2.imshow("kinect-registered", registered)
    #         cv2.setMouseCallback('kinect-registered', mouse_callback, self._registered)

    #         cv2.namedWindow('kinect-big_depth', cv2.CV_WINDOW_AUTOSIZE)
    #         big_depth = cv2.resize(self._big_depth.asarray(np.float32), 
    #             (int(1920 / 3), int(1082 / 3)))
    #         cv2.imshow("kinect-big_depth", big_depth)
    #         cv2.setMouseCallback('kinect-registered', mouse_callback, self._big_depth)

    #         cv2.namedWindow('kinect-rgbd', cv2.CV_WINDOW_AUTOSIZE)
    #         rgbd = self._color_depth_map.reshape(424, 512)
    #         cv2.imshow("kinect-rgbd", rgbd)
    #         # cv2.setMouseCallback('kinect-rgbd', mouse_callback, self._color_depth_map)

    #         self._listener.release(self._frames)

    #         key = cv2.waitKey(delay=1)
    #         if key == ord('q'):
    #             break

    def match_eval(self):
        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                # print(self.convert(x, y))
                # print(self._big_depth.asarray(np.float32)[1:-1, :].shape)
                # print("pixel x = {}, y = {}".format(x, y))
                depth_map = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
                depth = depth_map[y, x]
                # print(depth)
                cam_x = (x - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * depth
                cam_y = (y - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * depth
                print("camera frame x = {}, y = {}, z = {}".format(cam_x, cam_y, depth))
                point = np.ones((4,), dtype=np.float32)
                point[0] = cam_x
                point[1] = cam_y
                point[2] = depth
                # print(self._transformation)
                # print(self._transformation.dot(p))
                # print(np.linalg.inv(self._transformation).dot(point)[:3] / 1000)
                temp = np.linalg.inv(self._transformation).dot(point)[:3] / 1000
                ori = (0, 0, 0)
                z = np.array(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(ori))).reshape(3,3)[-1]
                # temp[2] += 0.04
                end_state = dict(position=temp - z * 0.02534,
                         orientation=p.getQuaternionFromEuler((0, 0, 0)))
                self._arm.reach_absolute(end_state)




        def mouse_callback_depth(event, x, y, flags, params):
            if event == 1:
                # print(self.convert(x, y))
                # print(self._big_depth.asarray(np.float32)[1:-1, :].shape)
                # print(x, y)
                print(params.shape)
                depth = params[y, x]
                print(depth)
        while True:
            
            color, ir, depth= self.snapshot()
            color = cv2.flip(color.asarray(), 1)
            ir = cv2.flip(ir.asarray(), 1)
            depth = cv2.flip(depth.asarray(), 1)

            cv2.namedWindow('kinect-rgb', cv2.CV_WINDOW_AUTOSIZE)
            rgb = cv2.resize(color, (int(1920), int(1080)))
            cv2.imshow("kinect-rgb", rgb)
            cv2.setMouseCallback('kinect-rgb', mouse_callback)

            cv2.namedWindow('kinect-big_depth', cv2.CV_WINDOW_AUTOSIZE)
            big_depth = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
            big_depth = (big_depth - 800) / (1500 - 800)
            cv2.imshow("kinect-big_depth", big_depth)
            cv2.setMouseCallback('kinect-big_depth', mouse_callback)

            # color[:, :, -2] = 0
            # cv2.namedWindow('kinect-r0', cv2.CV_WINDOW_AUTOSIZE)
            # cv2.imshow("kinect-r0", color)

            # color[:, :, -2] = 255
            # cv2.namedWindow('kinect-r255', cv2.CV_WINDOW_AUTOSIZE)
            # cv2.imshow("kinect-r255", color)

            color[:, :, -2] = (big_depth * 255).astype(np.uint8)
            cv2.namedWindow('kinect-map', cv2.CV_WINDOW_AUTOSIZE)
            cv2.imshow("kinect-map", color)

            # color[:, :, -3] = (big_depth * 255).astype(np.uint8)
            # color[:, :, -3] = big_depth
            # gray_image = np.array(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), dtype=np.float32)
            # gray_image += big_depth

            
            # cv2.setMouseCallback('kinect-map', mouse_callback)
            # cv2.namedWindow('kinect-depth', cv2.CV_WINDOW_AUTOSIZE)
            # cv2.imshow("kinect-depth", depth)
            # cv2.setMouseCallback('kinect-depth', mouse_callback_depth, depth)

            # big_depth = cv2.resize(self._big_depth.asarray(np.float32), 
            #     (int(1920 / 3), int(1082 / 3)))

            # cv2.namedWindow('kinect-registered', cv2.CV_WINDOW_AUTOSIZE)
            # registered = self._registered.asarray(np.uint8)
            # cv2.imshow("kinect-registered", registered)
            # cv2.setMouseCallback('kinect-registered', mouse_callback, self._registered)


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
        total = len(calibration_grid)
        count = 0
        for i, pos in enumerate(calibration_grid):
            print("{} / {}".format(i + 1, total))
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
            time.sleep(1)

            # Get the real position
            gripper_pos = np.array(self._arm.get_tool_pose()[0], 
                                   dtype=np.float32)
            gripper_ori = np.array(robot.get_tool_pose()[1], dtype=np.float32)
            trans_matrix = np.array(p.getMatrixFromQuaternion(gripper_ori)).reshape(3,3)
            marker_pos = gripper_pos + trans_matrix[-1] * 0.02534
            # Match with pattern location

            detected, pix = self._detect_center()

            # Skip if not found pattern
            if not detected:
                print('Pattern not detected. Skipping')
            else:
                print('Pattern detected! pos: {}'.format(pix))
                count = count + 1
                camera_points.append(pix)
                gripper_points.append(marker_pos * 1000)

        print("Solving parameters using {} images".format(count))
        # Solve for matrices
        retval, rvec, translation = cv2.solvePnP(
            np.array(gripper_points, dtype=np.float32), 
            # Only use the x, y to get R | T
            np.array(camera_points, dtype=np.float32), 
            self._intrinsics_RGB, 
            # Give none, assuming feeding in undistorted images 
            self._distortion_RGB)
            # self.d)

        rotation = cv2.Rodrigues(rvec)[0]
            
        data = dict(rotation=rotation,
                    translation=translation,)
                    # intrinsics=self.K,
                    # distortion=self.d)    
        print("saving data to" + pjoin(self._calib_directory, 
                '{}_'.format(self.__class__.__name__)))
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
        color, ir, depth = self.snapshot()
        color = cv2.flip(color.asarray(), 1)
        self._listener.release(self._frames)

        img = cv2.resize(color, (int(1920), int(1080)))

        foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
            img, self._board_size, None,
            cv2.CALIB_CB_ADAPTIVE_THRESH
        )
        if foundPattern:
            cv2.drawChessboardCorners(img, self._board_size, 
                  usbCamCornerPoints, foundPattern)

            cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 
                    usbCamCornerPoints, (11, 11), (-1, -1), 
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            usbCamCornerPoints = np.squeeze(usbCamCornerPoints)
            recognized_pix = np.mean(usbCamCornerPoints, axis=0)
            # print(tuple(recognized_pix.astype(int).tolist()))
            # depth = self._big_depth.asarray(np.float32)[1:-1, :][int(recognized_pix[1]), int(recognized_pix[0])]
            # # print(depth)
            # cam_x = (recognized_pix[0] - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * depth
            # cam_y = (recognized_pix[1] - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * depth
            # print("camera frame x = {}, y = {}, z = {}".format(cam_x, cam_y, depth))
            # cv2.circle(img, tuple(recognized_pix.astype(int).tolist()), 1, (0, 255, 0), 10)

            # cv2.imshow('kinect_calibrate', cv2.resize(img, (int(1920), int(1080))))
            # cv2.waitKey(3000)
            return True, recognized_pix
        else:
            # cv2.imshow('kinect_calibrate', cv2.resize(img, (int(1920), int(1080))))
            # cv2.waitKey(3000)
            return False, None

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
intrinsics_RGB = np.array([[1.0450585754139581e+03, 0., 9.2509741958808945e+02], 
                       [0., 1.0460057005089166e+03, 5.3081782987073052e+02], 
                       [0., 0., 1.]], dtype=np.float32)

distortion_RGB = np.array([ 1.8025470248423700e-02, -4.0380385825573024e-02,
       -6.1365440651701009e-03, -1.4119705487162354e-03,
       9.5413324012517888e-04 ], dtype=np.float32)

tracker = KinectTracker(robot, board_size=(4,4), itermat=(20, 20), intrinsics_RGB=intrinsics_RGB, distortion_RGB=distortion_RGB)

tracker.match_eval()


# np.set_printoptions(formatter={'float': lambda x: "{0:0.8f}".format(x)})

# end_state = dict(position=np.array([0.7, 0.0, 0.0], dtype=np.float32),
#                          orientation=p.getQuaternionFromEuler((0, 0, 0)))
#             # Move to target position
# robot.reach_absolute(end_state)
# gripper_pos = np.array(robot.get_tool_pose()[0], dtype=np.float32)
# print(gripper_pos)
# gripper_ori = np.array(robot.get_tool_pose()[1], dtype=np.float32)
# trans_matrix = np.array(p.getMatrixFromQuaternion(gripper_ori)).reshape(3,3)
# pos = gripper_pos + trans_matrix[-1] * 0.02534
# print(pos)