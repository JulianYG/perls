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
import time
import numpy as np

rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

arm = Robot(limb, gripper)

_REST_POSE = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
    'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

arm.set_init_positions(_REST_POSE)

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

cam = cv2.VideoCapture(0)

def calibrate_transformation(boardShape, checkerSize, gripperInitPos):

    s, foundPattern = False, False

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

    objectPoints = np.zeros((boardShape[1], boardShape[0], 3), dtype=np.float32)

    for i in range(boardShape[1]):
        for j in range(boardShape[0]):
            objectPoints[i, j] = [gripperInitPos[0] + i * checkerSize, 
                gripperInitPos[1] + j * checkerSize, 0.]

    objectPoints = objectPoints.reshape(numOfCornerPoints, 3).astype(np.float32)
    cornerPoints = cornerPoints.reshape(numOfCornerPoints, 2).astype(np.float32)

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

def move_to(x, y, z, t, iR):
    target_pos = calculate_position(x, y, t, iR)
    arm.reach_absolute({'position': (target_pos[0], target_pos[1], 
        z), 'orientation': (0, 1, 0, 0)})

def move_to_with_grasp(x, y, t, iR):
    move_to(x, y, 0.15, t, iR)
    time.sleep(0.2)
    move_to(x, y, 0.1, t, iR)
    time.sleep(0.3)
    arm.slide_grasp(0)

def move_to_with_grasp_and_lift(x, y, t, iR):
    move_to_with_grasp(x, y, t, iR)
    time.sleep(0.8)
    arm.set_init_positions(_REST_POSE)
    time.sleep(0.5)
    arm.slide_grasp(1)

tvec, invRotation = calibrate_transformation((9, 6), 0.026, (0.3045, -0.3623))

# print(tvec, invRotation)
# move_to_pixel(179, 401, 0.12, tvec, invRotation)

def mouse_callback(event, x, y, flags, params):
    if event == 1:
        move_to_with_grasp_and_lift(x, y, tvec, invRotation)


while 1:
    try:
        s, img = cam.read()
        if s:
            color_img = cv2.bilateralFilter(img, 9, 75, 75)
            color_img = cv2.fastNlMeansDenoisingColored(color_img, None, 10, 10, 7, 21)
            hsv_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)   # HSV image

            ###################################
            ### BLUE
            ###################################
            COLOR_MIN = np.array([110, 50, 50], dtype=np.uint8)
            COLOR_MAX = np.array([130, 255, 255], dtype=np.uint8)

            frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)     # Thresholding image
            ret, thresh = cv2.threshold(frame_threshed, 127, 255, 0)

            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            cubePoints = []

            for k, cnt in enumerate(contours):
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cubePoints.append((x + w / 2., y + h / 2.))

            for cp in cubePoints:
                move_to(cp[0], cp[1], 0.14, tvec, invRotation)
                time.sleep(1)

            cv2.namedWindow('grab-color', cv2.CV_WINDOW_AUTOSIZE)
            # cv2.setMouseCallback('grab-color', mouse_callback)
            cv2.imshow('grab-color', color_img)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()



