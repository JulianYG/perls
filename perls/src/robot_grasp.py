#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

def color_detection(img, boundaries=[
        ([17, 15, 100], [50, 56, 200]), # Red
        # ([86, 31, 4], [220, 88, 50]),   # Blue ok
        # ([25, 146, 190], [62, 174, 250]),    # Yellow none
        # ([0, 70, 35], [100, 200, 160])    # Green
    ]):

    # im = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)
    # hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)   # HSV image


    # COLOR_MIN = np.array([20, 100, 100], np.uint8)       # HSV color code lower and upper bounds
    # COLOR_MAX = np.array([30, 255, 255], np.uint8)       # color yellow 
    
    # frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)     # Thresholding image
    # imgray = frame_threshed
    # ret,thresh = cv2.threshold(frame_threshed, 127, 255, 0)
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # for cnt in contours:
    #     x,y,w,h = cv2.boundingRect(cnt)
    #     rospy.loginfo(x, y)
    #     cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
    # cv2.imshow("Show", im)
    # cv2.waitKey(1)

    # loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
     
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(img, lower, upper)
        output = cv2.bitwise_and(img, img, mask=mask)
     
        # show the images
        # cv2.imshow("images", np.hstack([img, output]))
        print(cv2.findChessboardCorners(img, (9, 6)), None)

        cv2.imshow('images', img)
        cv2.waitKey(1)


def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    def mouse_callback(event, x, y, flags, params):
        if event == 1:
            print(x, y)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
        cv2.namedWindow("cam-calibrate", cv2.CV_WINDOW_AUTOSIZE)
        cv2.imshow("cam-calibrate", cv_image)
        
        cv2.setMouseCallback('cam-calibrate', mouse_callback)
        cv2.waitKey(0)
    except CvBridgeError, err:
        rospy.logerr(err)
        return

    # color_detection(cv_image)
    

def main():
    """Camera Display Example
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return

    print("Initializing node... ")

    rospy.init_node('camera_display', anonymous=True)
    camera = intera_interface.Cameras()
    if not camera.verify_camera_exists('head_camera'):
        rospy.logerr("Invalid camera name, exiting the example.")
        return

    camera.start_streaming('head_camera')
    camera.set_callback('head_camera', show_image_callback,
        rectify_image=True)
    
    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()

if __name__ == '__main__':
    main()
