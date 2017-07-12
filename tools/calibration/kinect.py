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
from sensor_msgs.msg import CameraInfo, Image
import rospy
import rosparam


import sys, os
sys.path.append(os.path.abspath('../src/ros_/utils'))

# from camera import Kinect


dimension = (1920, 1080)
intrinsics = np.array([[1.0741796970429734e+03, 0., 9.3488214133804252e+02], 
                       [1.0640064260133906e+03, 0., 6.0428649994134821e+02], 
                       [0., 0., 1.]], dtype=np.float32)

distortion = np.array([ 3.4454149657654337e-02, 9.7555269933206498e-02,
       1.2981879029576470e-02, 2.7898744906562916e-04,
       -2.0379307133765162e-01 ], dtype=np.float32)

axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                   [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)



def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    def mouse_callback(event, x, y, flags, params):
        if event == 1:
            print(x, y, params[y, x])
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data)#, "bgr8")
        # f, points = cv2.findChessboardCorners(cv_image, (9, 6))

        # objp = np.zeros((6*9,3), np.float32)
        # objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # # print(cv2.findChessboardCorners(cv_image, (9, 6))[0])
        # if f:
        #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #     cv2.cornerSubPix(gray, 
        #         points, (11, 11), (-1, -1), 
        #         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))



        #     rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, points, intrinsics, distortion)
            
            

        #     imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, intrinsics, distortion)

        #     draw(cv_image, points, imgpts)
        # else:
        #     print('pattern not found')

        cv2.namedWindow("cam-calibrate", cv2.CV_WINDOW_AUTOSIZE)
        cv2.imshow("cam-calibrate", cv_image)

        cv2.setMouseCallback('cam-calibrate', mouse_callback, cv_image)
        cv2.waitKey(1)

        
    except CvBridgeError, err:
        rospy.logerr(err)
        return

    # color_detection(cv_image)
    

def main():
    """Camera Display Example
    """
 
    print("Initializing node... ")

    rospy.init_node('kinect', anonymous=True)
    # camera = Kinect(dimension, intrinsics, distortion)
    points = []
    info = dict(
            board_size=(9,6),
            num_of_points=54,
            directory='/home/cvgl_ros/Desktop/test_kinect',
            point_list=points,
            calibration_points=[]
            )

    rospy.Subscriber('/kinect2/hd/image_depth_rect',#'/kinect2/hd/image_color', 
            Image, show_image_callback)

    # camera.snapshot(show_image_callback)
    # camera.set_callback(camera_name, ,
    #     rectify_image=True)
    
    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()

if __name__ == '__main__':
    main()
