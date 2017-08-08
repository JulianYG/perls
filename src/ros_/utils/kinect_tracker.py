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
from intera_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

import time

sys.path.append(os.path.abspath('../'))

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest
)

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
LENGTH = 0.133

FINGER_OFFSET = 0.066

class KinectTracker():

    def __init__(self, r_mat, t_vec, k, d):

        if rospy.get_name() == '/unnamed':
            rospy.init_node('kinect_calibration')

        self._arm = intera_interface.Limb()
        self._gripper = intera_interface.Gripper()

        self._transformation = np.eye(4)
        self._transformation[:3, :3] = r_mat
        self._transformation[:3, 3] = t_vec


        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        # Turn on kinect
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
        self._intrinsics_RGB = k if k is not None else self._to_matrix(self._device.getColorCameraParams())
        self._distortion_RGB = d

        self._registration = Registration(self._device.getIrCameraParams(),
            self._device.getColorCameraParams())


        self._undistorted = Frame(512, 424, 4)
        self._registered = Frame(512, 424, 4)

        self._big_depth = Frame(1920, 1082, 4)
        self._color_depth_map = np.zeros((424, 512),  np.int32).ravel()

    def clean_shutdown(self):

        rospy.loginfo('Shutting down kinect calibration.')
        cv2.destroyWindow('kinect_calibrate')
        self._camera_man.unregister()
        if self.invR is None or self.T is None:
            print('computing...')
            self._compute()

    def reach_absolute(self, endState):
        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(*(endState['position'])),
                    orientation=Quaternion(*(endState['orientation'])),
                ),
            ),
        }
        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(poses["right"])
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq) # get IK response
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            self._arm.move_to_joint_positions(limb_joints)
            rospy.loginfo("Move to position succeeded")

        else:
            rospy.logerr("IK response is not valid")
            return False

    def convert(self, u, v):

        depth_map = cv2.flip(self._big_depth.asarray(np.float32)[1:-1, :], 1)
        depth_avg = depth_map[v, u] + KINECT_DEPTH_SHIFT

        print("== depth: {}".format(depth_avg))
        cam_x = (u - self._intrinsics_RGB[0, 2]) / self._intrinsics_RGB[0, 0] * depth_avg
        cam_y = (v - self._intrinsics_RGB[1, 2]) / self._intrinsics_RGB[1, 1] * depth_avg

        point = np.ones((4,), dtype=np.float32)
        point[0] = cam_x
        point[1] = cam_y
        point[2] = depth_avg

        target_point = np.linalg.inv(self._transformation).dot(point)[:3] / 1000
        print("== xyz in robot frame: {}".format(target_point * 1000))
        # print('getpoint3d: {}'.format(self._registration.getPointXYZ(Frame(512, 424, 4), u, v)))

        target_point[2] += LENGTH + 0.015
        # Prevent collision
        # target_point += 0.015
        print("== desired endeffector pos: {}".format(target_point * 1000))

        return target_point

    @staticmethod
    def _to_matrix(param):

        return np.array([
            [param.fx, 0, param.cx],
            [0, param.fy, param.cy],
            [0, 0, 1]],
            dtype=np.float32)

    def click_and_grasp(self):


        global mouseX, mouseY

        mouseX = 0
        mouseY = 0

        def mouse_callback(event, x, y, flags, params):
            if event == 1:
                mouseX, mouseY = x, y
                gripper_pos = self.convert(x, y)

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

    def move_to(self, x, y, z):

        end_state = dict(position=(x, y, z), orientation=(0, 1, 0, 0))
        self.reach_absolute(end_state)
        print("== move to {} success".format(self._arm.endpoint_pose()))

    def move_to_with_grasp(self, x, y, z, hover, dive):
        self._gripper.set_position(self._gripper.MAX_POSITION)
        self._arm.set_joint_position_speed(0.1)
        self.move_to(x, y, z + hover)
        time.sleep(0.7)
        self.move_to(x, y, z + dive)
        self._arm.set_joint_position_speed(0.03)
        self.move_to(x, y, z - FINGER_OFFSET)
        time.sleep(0.8)
        self._gripper.set_position(0)

    def move_to_with_lift(self, x, y, z,
        hover=0.4, dive=0.05, drop=True, drop_height=0.3):

        self.move_to_with_grasp(x, y, z, hover, dive)
        time.sleep(.75)
        self._arm.set_joint_position_speed(0.1)
        self.move_to(x, y, z + hover)
        time.sleep(0.2)

        self.move_to(x, y, z + drop_height)
        time.sleep(.8)

        if drop:
            self._gripper.set_position(self._gripper.MAX_POSITION)
            time.sleep(.5)



if __name__ == '__main__':
    if rospy.get_name() == '/unnamed':
        rospy.init_node('kinect_grasp')


    rotation_dir = pjoin('../../../tools/calibration/calib_data/kinect', 'KinectTracker_rotation.p')
    translation_dir = pjoin('../../../tools/calibration/calib_data/kinect', 'KinectTracker_translation.p')
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



    tracker = KinectTracker(r, t, intrinsics_RGB, distortion_RGB)

    tracker.click_and_grasp()
