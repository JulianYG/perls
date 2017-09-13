#!/usr/bin/env python
import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import Image
import tf.transformations as tff
import tf
from geometry_msgs.msg import Point, TransformStamped 
import pickle
from os.path import join as pjoin

CAMERA_PARAM_DIR = '../tools/calibration/calib_data/kinect/'

with open(pjoin(CAMERA_PARAM_DIR, 'intrinsics.p'), 'rb') as f:
    intrinsics_RGB = pickle.load(f)

with open(pjoin(CAMERA_PARAM_DIR, 'distortion.p'), 'rb') as f:
    distortion_RGB = pickle.load(f)

with open(pjoin(CAMERA_PARAM_DIR, 'KinectCalibrator_rotation.p'), 'rb') as f:
    rotation = pickle.load(f)

with open(pjoin(CAMERA_PARAM_DIR, 'KinectCalibrator_translation.p'), 'rb') as f:
    translation = pickle.load(f)

rmat = np.zeros((4, 4))
rmat[:3, :3] = rotation
rmat[3, 3] = 1.

translation = - rotation.T.dot(translation) / 1000.
rotation = tf.transformations.quaternion_from_matrix(rmat.T)


class GeneralPerception(object):

    def __init__(self):

        self.objects_receiver = rospy.Subscriber(
            "/tabletop_detector/object_markers", MarkerArray, self._get_objects)
        self.table_receiver = rospy.Subscriber(
            "/tabletop_detector/table_marker", Marker, self._get_table)
        self.depth_receiver = rospy.Subscriber(
            "/kinect2/hd/image_depth_rect", Image, self._get_depth)
        self.rgb_receiver = rospy.Subscriber(
            "/kinect2/hd/image_color_rect", Image, self._get_rgb)
        self.objects = None
        self.table = None
        self.depth = None
        self.rgb = None

    def update_rgbd(self):
        rospy.wait_for_message('/kinect2/hd/image_depth_rect', Image)
        rospy.wait_for_message('/kinect2/hd/image_color_rect', Image)

    def update_table(self):

        rospy.wait_for_message('/tabletop_detector/table_marker_aligned', Marker)

    def update_obj(self):
        rospy.wait_for_message('/tabletop_detector/object_markers_aligned', MarkerArray)

    def boot(self):

        rospy.wait_for_message('/kinect2/hd/image_depth_rect', Image)
        rospy.wait_for_message('/kinect2/hd/image_color_rect', Image)
        rospy.wait_for_message('/tabletop_detector/table_marker_aligned', Marker)
        rospy.wait_for_message('/tabletop_detector/object_markers_aligned', MarkerArray)

    def unregister(self):
        self.objects_receiver.unregister()
        self.table_receiver.unregister()
        self.depth_receiver.unregister()
        self.rgb_receiver.unregister()

    @staticmethod
    def transform(pose):
        t = tf.Transformer(True, rospy.Duration(10.0))

        k2o = TransformStamped()
        k2o.header.frame_id = '/kinect2_rgb_optical_frame'   
        k2o.child_frame_id = '/object'

        k2o.transform.translation.x = pose.position.x
        k2o.transform.translation.y = pose.position.y
        k2o.transform.translation.z = pose.position.z

        k2o.transform.rotation.w = pose.orientation.x
        k2o.transform.rotation.x = pose.orientation.y
        k2o.transform.rotation.y = pose.orientation.z
        k2o.transform.rotation.z = pose.orientation.w
        t.setTransform(k2o)

        b2k = TransformStamped()
        b2k.header.frame_id = '/base'
        b2k.child_frame_id = '/kinect2_rgb_optical_frame'

        b2k.transform.translation.x = translation[0]
        b2k.transform.translation.y = translation[1]
        b2k.transform.translation.z = translation[2]
        b2k.transform.rotation.w = rotation[3]
        b2k.transform.rotation.x = rotation[0]
        b2k.transform.rotation.y = rotation[1]
        b2k.transform.rotation.z = rotation[2]

        t.setTransform(b2k)
        return t.lookupTransform("/base", "/object", rospy.Time())

    def _get_objects(self, marker_array):
        self.objects = marker_array

    def _get_table(self, marker):
        self.table = marker

    def _get_depth(self, depth):
        self.depth = depth

    def _get_rgb(self, rgb):
        self.rgb = rgb

    def distance_from_table_center(self,obj):
        obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
        table_pos = [self.table.pose.position.x,self.table.pose.position.y,self.table.pose.position.z]
        dist = math.sqrt(sum([(o-t)**2 for o,t in zip(obj_pos,table_pos)]))
        return dist

    def find_central_object(self):
        obj_with_dists = [(obj,self.distance_from_table_center(obj)) for obj in self.objects.markers]
        min_dist_obj = min(obj_with_dists, key=lambda o:o[1])
        return min_dist_obj[0]

def convert_point(x,y,z):
    u = x / z * intrinsics_RGB[0, 0] +  intrinsics_RGB[0, 2]
    v = y / z * intrinsics_RGB[1, 1] +  intrinsics_RGB[1, 2]
    return int(u), int(v)


if __name__ == '__main__':

    rospy.init_node('her')
    receiver = GeneralPerception()
    receiver.boot()
    
    img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
    for i, obj in enumerate(receiver.objects.markers):

        pos, orn = GeneralPerception.transform(obj.pose)
        obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]

        obj_size = [obj.scale.x,obj.scale.y,obj.scale.z]
        print('position: {}, orn: {}, size: {}, number: {}'.format(pos, orn, obj_size, i))
        u1,v1 = convert_point(obj_pos[0]-obj_size[0]/2,obj_pos[1]-obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
        u2,v2 = convert_point(obj_pos[0]+obj_size[0]/2,obj_pos[1]+obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
        cv2.rectangle(img, (u1, v1), (u2, v2), (0, 0, 255), 2)

    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    receiver.unregister()
