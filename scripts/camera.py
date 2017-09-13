#!/usr/bin/env python

import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def callback(img_data):
    """
    Grasp objects based on mouse click position
    on HD RGB image.
    :return: None
    """

    cv_image = CvBridge().imgmsg_to_cv2(img_data, '8UC1')  # Use 'bgr8' for color

    # undistorted_color = cv2.undistort(cv_image, self._intrinsics_RGB, self._distortion_RGB)
    # color = cv2.flip(cv_image, 1)

    foundPattern, irCornerPoints = cv2.findChessboardCorners(
        cv_image, (4, 4), None,
        cv2.CALIB_CB_ADAPTIVE_THRESH
    )

    if foundPattern:

        cv2.cornerSubPix(cv_image, 
            irCornerPoints, (11, 11), (-1, -1), 
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(cv_image, (4, 4), irCornerPoints, foundPattern)

    cv2.namedWindow('kinect_view', cv2.CV_WINDOW_AUTOSIZE)
    cv2.imshow('kinect_view', cv_image)      
    cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('view')

    # _rgb = rospy.Subscriber(
        # '/kinect2/hd/image_color_rect',
        # Image, callback)

    _rgb = rospy.Subscriber(
        '/kinect2/sd/image_ir_rect',
        Image, callback)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        cv2.destroyWindow('kinect_view')
        sys.exit(0)

