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
_ALPHA = 0.026 / 21

# Camera matrix
_K = np.array([
    [600.153387, 0, 315.459915], 
    [0, 598.015225, 222.933946], 
    [0,          0,          1]
    ], np.float32)

_invK = np.linalg.inv(_K)

# Distortion
_D = np.array([0.147084, -0.257330, 
    0.003032, -0.006975, 0.000000], np.float32)

def calibrate_transformation(boardShape, checkerSize, gripperInitPos):

    cam, s, foundPattern = cv2.VideoCapture(0), False, False

    # Loop until read image
    while not s:
        s, img = cam.read()

    # Loop until found checkerboard pattern
    while not foundPattern:
        foundPattern, cornerPoints = cv2.findChessboardCorners(
            img, boardShape, None, 
            cv2.CALIB_CB_ADAPTIVE_THRESH
        )

    numOfCornerPoints = cornerPoints.shape[0]

    objectPoints = np.zeros((boardShape[0], boardShape[1], 3), dtype=np.float32)

    for i in range(boardShape[0]):
        for j in range(boardShape[1]):
            objectPoints[i, j] = [gripperInitPos[0] + i * checkerSize, 
                gripperInitPos[1] + j * checkerSize, 0]

    objectPoints = objectPoints.reshape(numOfCornerPoints, 3).astype(np.float32)
    cornerPoints = cornerPoints.reshape(numOfCornerPoints, 2).astype(np.float32)

    print(objectPoints)

    retval, rvec, tvec = cv2.solvePnP(
        objectPoints, cornerPoints,
        _K, _D)

    rotMatrix = cv2.Rodrigues(rvec)[0]
    invRotation = np.linalg.inv(rotMatrix)

    return tvec, invRotation

def calculate_position(x, y, t, invR):
    # Change the pixel point here (in form [u,v,1])
    tempMat = invR * _invK

    pixelPoint = np.array([x, y, 1]).astype(np.float32) 
    tempMat2 = tempMat.dot(pixelPoint)
    tempMat3 = invR.dot(np.reshape(t, 3))

    # approx height of each block + 
    # (inv Rotation matrix * inv Camera matrix * point) 
    s = .026 + tempMat3[2]

    # inv Rotation matrix * tvec
    s /= tempMat2[2] 

    # s * [u,v,1] = M(R * [X,Y,Z] - t)  
    # ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
    temp = _invK.dot(s * pixelPoint)
    temp2 = temp - np.reshape(t, 3)
    realworldCoords = invR.dot(temp2)

    return realworldCoords

def move_to_pixel(x, y, z, t, iR):
    target_pos = calculate_position(x, y, t, iR)
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        z), 'orientation': (0, 1, 0, 0)})


tvec, invRotation = calibrate_transformation((9, 6), 0.026, (0.2757, -0.3494))

# print(tvec, invRotation)
# move_to_pixel(268, 414, 0.12, tvec, invRotation)




