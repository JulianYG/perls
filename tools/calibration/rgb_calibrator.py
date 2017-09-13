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
import tf.transformations as ttf
sys.path.append(os.path.abspath(os.path.join(__file__, '../../../src/pyrobots')))

from sawyer import SawyerArm

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

KINECT_DEPTH_SHIFT = -22.54013555237548
GRIPPER_SHIFT = 0.0251
# dimension = (1920, 1080)


class KinectRGBCalibrator():

    def __init__(self, robot, 
        board_size=(2,2), itermat=(8, 9)):

        self._board_size = board_size

        self._arm = robot
        self._grid = itermat

        self._calib_directory = os.path.join(
            os.path.dirname(__file__), 
            'calib_data', 'kinect')

        # Calibrated by IAI_Kinect
        with open(pjoin(self._calib_directory, 'RGB_intrinsics.p'), 'rb') as f:
            self._intrinsics_RGB = pickle.load(f)

        with open(pjoin(self._calib_directory, 'RGB_distortion.p'), 'rb') as f: 
            self._distortion_RGB = pickle.load(f)

        self._transformation = np.zeros((4, 4), dtype=np.float32)
        self._transformation[3, 3] = 1

        self.global_x, self.global_y = 0, 0
        
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

        # Read from kinect directly
        # self._intrinsics_RGB = self._to_matrix(
        #     self._device.getColorCameraParams())

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

    
    def match_eval(self):

    	LENGTH = 0.19406304511449886

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                self.global_x, self.global_y = x, y

                depth_map = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
                depth_avg = depth_map[y, x] + KINECT_DEPTH_SHIFT
                print("== depth: {}".format(depth_avg))
                cam_x = (x - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * depth_avg
                cam_y = (y - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * depth_avg

                point = np.ones((4,), dtype=np.float32)
                point[0] = cam_x
                point[1] = cam_y
                point[2] = depth_avg

                ori = ttf.quaternion_from_euler(np.pi, 0, 0)
                z = ttf.quaternion_matrix(ori)[:3, :3][-1]

                target_point = np.linalg.inv(self._transformation).dot(point)[:3] / 1000
                print("== xyz in robot frame: {}".format(target_point * 1000))
                print(z)
                target_point = target_point - z * LENGTH  #GRIPPER_SHIFT

                print("== desired endeffector pos: {}".format(target_point * 1000))

                self._arm.tool_pose = (target_point, ori)
                time.sleep(1)

                print("== move to {} success".format(self._arm.tool_pose[0]))
                print(self._arm.tool_pose[0])

        cv2.namedWindow('kinect-rgb', cv2.CV_WINDOW_AUTOSIZE)
        cv2.setMouseCallback('kinect-rgb', mouse_callback)

        while True:
            color, _, _ = self.snapshot()
            undistorted_color = cv2.undistort(color.asarray(), self._intrinsics_RGB, self._distortion_RGB)
            color = cv2.flip(undistorted_color, 1)
            depth_map = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
            
            cv2.circle(color, (self.global_x, self.global_y), 1, (0, 255, 0), 10)
            rgb = cv2.resize(color, (int(1920), int(1080)))
            cv2.imshow("kinect-rgb", rgb)

            self._listener.release(self._frames)
            
            key = cv2.waitKey(delay=1)
            if key == ord('q'):
                break

    def find_constant(self):

        self._grid = (3, 3)
        origin = np.array([0.43489, -0.2240, 0.1941], dtype=np.float32)
        orn = np.array([0, 0, 0, 1], dtype=np.float32)

        calibration_grid = np.zeros((self._grid[0] * self._grid[1], 3), np.float32)
        calibration_grid[:, :2] = np.mgrid[0: self._grid[0], 
                                           0: self._grid[1]].T.reshape(-1, 2) * 0.033
        # And randomness to z                                  
        calibration_grid[:, -1] += np.random.uniform(-0.08, 0.2, 
            self._grid[0] * self._grid[1])

        error_list = []
        errors = []
        counts = []
        itrs = np.arange(0, 30.0, 5.0)
        for j in range(len(itrs)):
        	errors.append(np.zeros((3,)))
        	counts.append(0)
        for i, pos in enumerate(calibration_grid):
            print("{} / 9".format(i + 1))
            target = origin + pos

            # Add randomness to orientation
            orn = ttf.quaternion_from_euler(*np.random.uniform(
                [-np.pi/12., -np.pi/12., -np.pi/6.],[np.pi/12., np.pi/12., np.pi/6.]))

            # Move to target position
            self._arm.tool_pose = (tuple(target), orn)
            time.sleep(2)
            not_found = 0

            for j, shift in enumerate(itrs):

                KINECT_DEPTH_SHIFT = -shift
                inner_count = 0

                if not_found > 5:
                    break
                while inner_count < 30:

                    if not_found > 5:
                        break

                    inner_count = inner_count + 1
                    color, _, _ = self.snapshot()
                    color = cv2.flip(color.asarray(), 1)

                    detected, pix = self._detect_center(color)
                    if detected:

                        counts[j] = counts[j] + 1
                        x, y = pix
                        depth_map = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
                        depth = depth_map[int(y + 0.5), int(x + 0.5)] + KINECT_DEPTH_SHIFT

                        cam_x = (x - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * depth
                        cam_y = (y - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * depth

                        point = np.ones((4,), dtype=np.float32)
                        point[0] = cam_x
                        point[1] = cam_y
                        point[2] = depth
                        
                        temp = np.linalg.inv(self._transformation).dot(point)[:3] / 1000
                        ori = np.array(self._arm.tool_pose[1], dtype=np.float32)
                        z = ttf.quaternion_matrix(ori)[:3, :3][-1]

                        estimated_gripper_pos = temp - z * GRIPPER_SHIFT
                        ground_truth = np.array(self._arm.tool_pose[0], dtype=np.float32)
                        errors[j] += np.sqrt((estimated_gripper_pos - ground_truth) * (estimated_gripper_pos - ground_truth))
                    else:
                    	not_found = not_found + 1

                    self._listener.release(self._frames)

        for j, shift in enumerate(itrs):
        	print("shift: {}, err avg: {}".format(shift, errors[j] / counts[j]))

    def turn_off(self):
        self._device.stop()
        self._device.close()
        self.camera_on = False

    def track(self):

        rotation_dir = pjoin(self._calib_directory, 'KinectRGBCalibrator_rotation.p')
        translation_dir = pjoin(self._calib_directory, 'KinectRGBCalibrator_translation.p')

        if os.path.exists(rotation_dir) and os.path.exists(translation_dir):
            
            with open(rotation_dir, 'rb') as f:
                rotation = pickle.load(f)

            with open(translation_dir, 'rb') as f:
                translation = np.squeeze(pickle.load(f))
        
            self._transformation[:3, :3] = rotation
            self._transformation[:3, 3] = translation
            return

        # Reasonable starting position
        origin = np.array([0.43489, -0.2240, 0.1241], dtype=np.float32)
        # origin = np.array([0.43489, -0.2240, 0.00941], dtype=np.float32)
        orn = np.array([0, 1, 0, 0], dtype=np.float32)

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
            orn = ttf.quaternion_from_euler(*np.random.uniform(
                [-np.pi/15., -np.pi/15., -np.pi/8.], [np.pi/15., np.pi/15., np.pi/8.]))

            # Move to target position
            self._arm.tool_pose = (tuple(target), orn)

            # Wait till stabilize
            time.sleep(1.5)

            # Get the real position
            gripper_pos = np.array(self._arm.tool_pose[0], 
                                   dtype=np.float32)
            gripper_ori = np.array(self._arm.tool_pose[1], dtype=np.float32)
            trans_matrix = ttf.quaternion_matrix(gripper_ori)[:3, :3]
            marker_pos = gripper_pos + trans_matrix[-1] * GRIPPER_SHIFT

            # Match with pattern location
            color, ir, depth = self.snapshot()

            color = cv2.flip(color.asarray(), 1)
            self._listener.release(self._frames)

            detected, pix = self._detect_center(color)

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

            # Only use the x, y to get R | T;
            # depth z is separate
            np.array(camera_points, dtype=np.float32), 
            self._intrinsics_RGB, 

            # Give none, assuming feeding in undistorted images 
            self._distortion_RGB)

        rotation = cv2.Rodrigues(rvec)[0]
            
        data = dict(rotation=rotation,
                    translation=translation,)

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

    def _detect_center(self, color):

        img = cv2.resize(color, (int(1920), int(1080)))
        foundPattern, rgbCornerPoints = cv2.findChessboardCorners(
            img, self._board_size, None,
            cv2.CALIB_CB_ADAPTIVE_THRESH
        )
        if foundPattern:

            cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 
                    rgbCornerPoints, (11, 11), (-1, -1), 
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            rgbCornerPoints = np.squeeze(rgbCornerPoints)
            recognized_pix = np.mean(rgbCornerPoints, axis=0)
            return True, recognized_pix
        else:
            return False, None

    @staticmethod
    def _to_matrix(param):

        return np.array([
            [param.fx, 0, param.cx], 
            [0, param.fy, param.cy], 
            [0, 0, 1]], 
            dtype=np.float32)

if rospy.get_name() == '/unnamed':
    rospy.init_node('kinect_rgb_calibrator')

limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.1)
robot = SawyerArm(False)

tracker = KinectRGBCalibrator(
    robot, board_size=(4,4), itermat=(15, 15))
np.set_printoptions(formatter={'float': lambda x: "{0:0.8f}".format(x)})

tracker.match_eval()
# tracker.find_constant()
# tracker.turn_off()

