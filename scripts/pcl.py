#!/usr/bin/env python
import rospy
import cv2
import sys
import os

import numpy as np
from cv_bridge import CvBridge
from perls.pcl_segment import PCLSegment

if __name__ == '__main__':

    rospy.init_node('pcl')

    receiver = PCLSegment()
    receiver.boot()
    img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')#[400:1000, 450:1300]

    for i, obj in enumerate(receiver.objects.markers):

        pos, orn = PCLSegment.transform(obj.pose)
        obj_pos = np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
        obj_size = np.array([obj.scale.x, obj.scale.y, obj.scale.z]) / 2.
        print('position: {}, orn: {}, number: {}'.format(pos, orn, i))

        u1, v1 = PCLSegment.convert(*(obj_pos + obj_size) / 2.)
        u2, v2 = PCLSegment.convert(*(obj_pos - obj_size) / 2.)
        cv2.rectangle(img, (u1, v1), (u2, v2), (0, 0, 255), 2)

    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    receiver.unregister()
