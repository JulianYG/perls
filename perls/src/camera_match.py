#!/usr/bin/env python
from __future__ import print_function

import sys, os
import rospy
import cv2
import intera_interface
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'ros_'))
from robot import Robot

import numpy as np
from numpy.linalg import LinAlgError
import scipy.linalg as sn

rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

arm = Robot(limb, gripper)

rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
    'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

arm.set_init_positions(rest_pose)

_INIT_POS = np.array(arm.get_tool_pose()[0][:2])
_PXL = (457, 192)

_ALPHA = 0.026 / 21

# Camera matrix
_K = np.array([[600.153387, 0, 315.459915], [0, 598.015225, 222.933946], [0, 0, 1]], np.float32)

# Distortion
_D = np.array([0.147084, -0.257330, 0.003032, -0.006975, 0.000000], np.float32)

def pixel_diff_to_pos(curr_pos, curr_pxl, targ_pxl):
    dist = (np.array(targ_pxl) - np.array(curr_pxl)) * _ALPHA
    return curr_pos + dist[::-1]

def get_current_pixel(curr_pos):
    pxl = ((curr_pos - _INIT_POS)[::-1] / _ALPHA).astype(np.int32) + _PXL
    return (pxl[0], pxl[1])

def move_to_pixel_without_reset(x, y):

    # Ignore z-axis value
    gripper_pos = np.array(arm.get_tool_pose()[0][:2])
    curr_pxl = get_current_pixel(gripper_pos)

    target_pos = pixel_diff_to_pos(gripper_pos, curr_pxl, (x, y))
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        0.12), 'orientation': (0, 1, 0, 0)})

def move_to_pixel_with_reset(x, y):

    arm.set_init_positions(rest_pose)
    target_pos = pixel_diff_to_pos(gripper_pos, _PXL, (x, y))
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        0.12), 'orientation': (0, 1, 0, 0)})

def calibrate_transformation(imgPoints, objectPoints, K, D):

    # pts = np.array(pts)

    # # K^-1 * [u, v, 1]' = x * [X, Y, Z, 1]'
    # p = pts[:, 0].T 

    # p[0, :] -= 320
    # p[1, :] = 240 - p[1, :]

    # global xx
    # xx = p

    # d = np.matmul(np.linalg.inv(K), p)

    # C = np.vstack([pts[:, 1].T, 
    #     np.ones(len(pts), dtype=np.float32)])

    # # x = d \ C, x * c = d
    # # (xc)' = d', c'x' = d', x = (d'/c')'
    # try:
    #     # M = np.linalg.solve(C.T, d.T)[0].T
    #     M = d.dot(sn.pinv(C))
    # except LinAlgError:
    #     M = np.linalg.lstsq(C.T, d.T)[0].T
    # return M, M[:3, :3], M[:, 3]

    retval, rvec, tvec = cv2.solvePnP(objectPoints, imgPoints.astype(np.float32), K, D)

    rotMatrix = cv2.Rodrigues(rvec)[0]

    invRot = np.linalg.inv(rotMatrix)
    invCamera = np.linalg.inv(K)

    return tvec, invRot, invCamera

# move_to_pixel_without_reset(494, 392)

def calculate_position(x, y, t, iR, iC):

    # A = _K.dot(M)
    # b = np.array([[x - 320], [240 - y], [1]])

    # # a x = p, solve for x
    # return sn.pinv(A).dot(b)

    uvPoint = np.array([x,y,1]).astype(np.float32) # Change the pixel point here (in form [u,v,1])
    tempMat = iR * iC
    tempMat2 = tempMat.dot(uvPoint)
    tempMat3 = iR.dot(np.reshape(t, 3))

    s = .09 + tempMat3[2] # .09 + (inv Rotation matrix * inv Camera matrix * point) -> .09 is approx height of each block
    s /= tempMat2[2] # inv Rotation matrix * tvec


    #s * [u,v,1] = M(R*[X,Y,Z] - t)  ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
    temp = iC.dot(s * uvPoint)
    temp2 = temp - np.reshape(t, 3)

    realworldCoords = iR.dot(temp2)
    return realworldCoords

tvec, invR, invC = calibrate_transformation(
    np.array([[171, 218],
              [274, 223],
              [351, 219],
              [415, 221],
              [171, 319],
              [272, 318],
              [353, 315],
              [445, 318],
              [173, 404],
              [267, 413],
              [355, 411],
              [445, 414]]),
    np.array(
        [[0.4957, -0.1641, 0.0945], 
        [0.4947, -0.0416, 0.0920], 
        [0.4921,0.0445,0.0921],
        [0.4835,0.1466,0.0895],
        [0.6132,-0.1623,0.0884],
        [0.6070,-0.0430,0.0922],
        [0.6042,0.0418,0.0916],
        [0.5983,0.1526,0.0873],
        [0.6758,-0.1606,0.0890],
        [0.717,-0.0451,0.0878],
        [0.7135,0.0443,0.0966],
        [0.7175, 0.1570, 0.0880]]),
    _K, _D
)


def move_to_pixel(x, y):
    target_pos = calculate_position(x, y, tvec, invR, invC)
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        0.12), 'orientation': (0, 1, 0, 0)})

move_to_pixel(271, 317)
# print(calculate_position(355, 411, tvec, invR, invC))

# arm.reach_absolute({'position': (0.44989269453228237, 0.15755170628471973, 0.2), 'orientation': (0, 1, 0, 0)})
# class image_converter:

#     def __init__(self):
        
#         self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)

#     def callback(self,data):

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         (rows,cols,channels) = cv_image.shape
#         if cols > 60 and rows > 60:
#             cv2.circle(cv_image, (50, 50), 10, 255)

#         cv2.imshow("Image window", cv_image)
#         cv2.waitKey(3)

#         try:

#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

#         except CvBridgeError as e:
#             print(e)


