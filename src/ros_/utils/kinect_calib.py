# coding: utf-8




# while True:
#     frames = listener.waitForNewFrame()

#     color = frames["color"]
#     ir = frames["ir"]
#     depth = frames["depth"]

#     registration.apply(color, depth, undistorted, registered,
#                        bigdepth=bigdepth,
#                        color_depth_map=color_depth_map)

#     # NOTE for visualization:
#     # cv2.imshow without OpenGL backend seems to be quite slow to draw all
#     # things below. Try commenting out some imshow if you don't have a fast
#     # visualization backend.
#     cv2.imshow("ir", ir.asarray() / 65535.)
#     cv2.imshow("depth", depth.asarray() / 4500.)
#     cv2.imshow("color", cv2.resize(color.asarray(),
#                                    (int(1920 / 3), int(1080 / 3))))
#     cv2.imshow("registered", registered.asarray(np.uint8))

#     if need_bigdepth:
#         cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
#                                           (int(1920 / 3), int(1082 / 3))))
#     if need_color_depth_map:
#         cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

#     listener.release(frames)

#     key = cv2.waitKey(delay=1)
#     if key == ord('q'):
#         break

# device.stop()
# device.close()

# sys.exit(0)




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

import rospy
import rosparam
import intera_interface
import time


class KinectTracker():


    def __init__(self, robot, dimension='hd',
        board_size=(2,2), itermat=(8, 9),
        K=None, D=None, calib='../../../tools/calibration/calib_data/kinect'):

        if dimension == 'hd':
            self._dim = (1920, 1080)
        elif dimension == 'qhd':
            self._dim = (960, 540)
        else:
            self._dim == (512, 424)

        self._board_size = board_size
        self._checker_size = 0.0274


        self._arm = robot
        self._grid = itermat

        self._calib_directory = calib

        self.K = K
        self.d = D
        self.calibration_points = []

        self._camera = self._get_device()

        self._listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)
        
        self._registration = self._boot()

        self.track()




    def _boot(self):

        self._undistorted = Frame(512, 424, 4)
        self._registered = Frame(512, 424, 4)

        self._bigdepth = Frame(1920, 1082, 4)
        self._color_depth_map = np.zeros((424, 512),  np.int32).ravel()

        # Register listeners
        self._device.setColorFrameListener(listener)
        self._device.setIrAndDepthFrameListener(listener)

        self._device.start()

        # NOTE: must be called after device.start()
        if self.K is None or self.d is None:

            registration = Registration(self._device.getIrCameraParams(),
                            self._device.getColorCameraParams())
        else:
            IrParams = IrCameraParams(self.K, ...)
            colorParams = ColorCameraParams(self.K, ...)

            registration = Registration(IrParams, colorParams)

        return registration

    def read_image(self):

        frames = self._listener.waitForNewFrame()
        color, ir, depth = frames['color'], frames['ir'], frames['depth']
        
        self._registration.apply(color, depth, 
            self._undistorted,
            self._registered, 
            bigdepth=self._bigdepth, 
            color_depth_map=self._color_depth_map)

    


    def _get_object_points(self, size):
        """
        Return a numpy array of object points in shape [num, 3], float32
        The object points represent the corner points on checkerboard
        in real world frame, origin from the top left corner of 
        the checkerboard
        """
        raw_points = np.zeros(
            (self._board_size[1], self._board_size[0], 3), 
            dtype=np.float32)

        for i in range(self._board_size[1]):
            for j in range(self._board_size[0]):
                # Use 0 for z as for lying on the plane
                raw_points[i, j] = [i * self._checker_size, 
                    j * self._checker_size, 0.]

        object_points = np.reshape(raw_points, 
            (self._board_size[0] * self._board_size[1], 3)).astype(np.float32)

        # Correspond correct number of points with sampled points
        return [object_points] * size

    def track(self):

        invRotation_dir = pjoin(self._calib_directory, 'Tracker_inverseRotation.p')
        translation_dir = pjoin(self._calib_directory, 'Tracker_translation.p')

        if os.path.exists(invRotation_dir) and os.path.exists(translation_dir):
            
            with open(invRotation_dir, 'rb') as f:
                ir = pickle.load(f)
            self.invR = ir

            with open(translation_dir, 'rb') as f:
                t = pickle.load(f)
            self.T = t
        
            return t, ir

        origin = np.array([0.3690, -0.2566, 0.27], dtype=np.float32)
        orn = np.array([0, 0, 0, 1], dtype=np.float32)

        calibration_grid = np.zeros((self._grid[0] * self._grid[1], 3), np.float32)
        calibration_grid[:, :2] = np.mgrid[0: self._grid[0], 
                                           0: self._grid[1]].T.reshape(-1, 2) * 0.033
        # And randomness to z                                  
        calibration_grid[:, -1] += np.random.uniform(-0.08, 0.2, 
            self._grid[0] * self._grid[1])

        image_points, gripper_points = [], []

        for pos in calibration_grid:

            # Set position to reach for
            target = origin + pos

            # Add randomness to orientation
            # target_orn = orn + np.random.normal(0, 0.05, 4)
            # target_orn /= np.sqrt(np.sum(target_orn ** 2))

            end_state = dict(position=tuple(target),
                         orientation=(0,0,0,1))
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
                continue

            image_points.append(pix)
            gripper_points.append(gripper_pos)

        object_points = self._get_object_points(len(image_points))

        if self.K is None or self.d is None:
            _, self.K, self.d, _, _ = cv2.calibrateCamera(
                object_points, self.calibration_points, self._camera.dimension)
            self.d = np.squeeze(self.d)

        # Solve for matrices
        retval, rvec, tvec = cv2.solvePnP(
            np.array(gripper_points, dtype=np.float32), 
            np.array(image_points, dtype=np.float32), 
            self.K, 
            self.d)

        rotMatrix = cv2.Rodrigues(rvec)[0]
        invRotation = np.linalg.inv(rotMatrix)
            
        data = dict(inverseRotation=invRotation,
                    translation=tvec,
                    intrinsics=self.K,
                    distortion=self.d)

        for name, mat in data.items():
            with open(pjoin(self._calib_directory, 
                '{}_{}.p'.format(self.__class__.__name__, name)), 'wb') as f:
                pickle.dump(mat, f)

        self.T = tvec
        self.invR = invRotation

        return tvec, invRotation

    @staticmethod
    def convert(u, v, z, K, T, invR):

        # Pixel value
        p = np.array([u, v, 1], dtype=np.float32)

        tempMat = invR * K
        tempMat2 = tempMat.dot(p)
        tempMat3 = invR.dot(np.reshape(T, 3))

        # approx height of each block + 
        # (inv Rotation matrix * inv Camera matrix * point)
        # inv Rotation matrix * tvec
        s = (z + tempMat3[2]) / tempMat2[2]

        # s * [u,v,1] = M(R * [X,Y,Z] - t)  
        # ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
        temp = np.linalg.inv(K).dot(s * p)
        temp2 = temp - np.reshape(T, 3)
        gripper_coords = invR.dot(temp2)

        return gripper_coords

    def _detect_center(self):

        # Take picture
        img = self._camera.snapshot()
        
        # Look for pattern
        foundPattern, usbCamCornerPoints = cv2.findChessboardCorners(
            img, self._board_size, None, 
            cv2.CALIB_CB_ADAPTIVE_THRESH
        )
        
        if self.debug:
            plt.imshow(img)
            plt.show()  

        if foundPattern:
            cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 
                    usbCamCornerPoints, (11, 11), (-1, -1), 
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            usbCamCornerPoints = np.squeeze(usbCamCornerPoints)
            recognized_pix = np.mean(usbCamCornerPoints, axis=0)

            cv2.drawChessboardCorners(img, self._board_size, 
                usbCamCornerPoints, foundPattern)
            self.calibration_points.append(usbCamCornerPoints)
            return True, recognized_pix
        else:
            return False, None

    def match_eval(self):

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                print((x, y), self.convert(x, y, 
                    self.z_offset, self.K, self.T, self.invR))
        img = self._camera.snapshot()
        while True:
            try:
                cv2.namedWindow('match_eval', cv2.CV_WINDOW_AUTOSIZE)
                cv2.setMouseCallback('match_eval', mouse_callback)
                # # Remove distortion
                # rectified_image = cv2.undistort(img, self.K, 
                #   self.d)
                cv2.imshow('match_eval', img)

                # Update per millisecond
                cv2.waitKey(1)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()

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

tracker = KinectTracker(robot, board_size= (9,6), itermat=(3,4), K=intrinsics,
    D=distortion)

tracker.match_eval()



