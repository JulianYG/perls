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

from os.path import join as pjoin

CAMERA_PARAM_DIR = '../tools/calibration/calib_data/kinect/'

with open(pjoin(CAMERA_PARAM_DIR, 'intrinsics.p'), 'rb') as f:
    intrinsics_RGB = pickle.load(f)

with open(pjoin(CAMERA_PARAM_DIR, 'distortion.p'), 'rb') as f:
    distortion_RGB = pickle.load(f)


class GeneralPerception(object):
    def __init__(self, task):
        self.task = task
        self.objects_receiver = rospy.Subscriber("/tabletop_detector/object_markers", MarkerArray, self.get_objects)
        self.table_receiver = rospy.Subscriber("/tabletop_detector/table_marker", Marker, self.get_table)
        self.depth_receiver = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.get_depth)
        self.rgb_receiver = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.get_rgb)
        self.objects = None
        self.table = None
        self.depth = None
        self.rgb = None
        # self.wait_to_receive()

    def wait_to_receive(self):
        if self.table is None:
            rospy.wait_for_message('/tabletop_detector/table_marker_aligned',Marker)
        if self.objects is None:
            rospy.wait_for_message('/tabletop_detector/object_markers_aligned',MarkerArray)
        if self.depth is None:
            rospy.wait_for_message('/kinect2/hd/image_depth_rect',Image)
        if self.rgb is None:
            rospy.wait_for_message('/kinect2/hd/image_color_rect',Image)

    def unregister(self):
        self.objects_receiver.unregister()
        self.table_receiver.unregister()
        self.depth_receiver.unregister()
        self.rgb_receiver.unregister()

    def transform(self, pose):
        t = tf.Transformer(True, rospy.Duration(10.0))
        m = TransformStamped()
        m.header.frame_id = "/kinect2_ir_optical_frame"
        m.child_frame_id = "/grasp"
        m.transform.translation.x = pose.position.x
        m.transform.translation.y = pose.position.y
        m.transform.translation.z = pose.position.z
        m.transform.rotation.w = pose.orientation.x
        m.transform.rotation.x = pose.orientation.y
        m.transform.rotation.y = pose.orientation.z
        m.transform.rotation.z = pose.orientation.w
        t.setTransform(m)
        m = TransformStamped()
        m.header.frame_id = "/base"
        m.child_frame_id = "/kinect2_ir_optical_frame"

        # Values from readme 
        m.transform.translation.x = 1.03092698359  
        m.transform.translation.y = 0.09316863517
        m.transform.translation.z = 0.76020784767
        m.transform.rotation.w = 0.703789173
        m.transform.rotation.x = 0.710379928
        m.transform.rotation.y = 0.00641113624
        m.transform.rotation.z = 0.000230939802
        t.setTransform(m)
        pos, quat = t.lookupTransform("/base", "/grasp", rospy.Time())
        return pos,quat

    def get_objects(self, marker_array):
        # print "got objects"
        self.objects = marker_array

    def get_table(self, marker):
        # print "Got table"
        self.table = marker

    def get_depth(self, depth):
        # print "Got Depth"
        self.depth = depth

    def get_rgb(self, rgb):
        # print "Got RGB"
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

    def get_cubes(self):
        self.wait_to_receive()
        if self.task == "stacking":
            obj_with_dists = [(obj,self.distance_from_table_center(obj)) for obj in self.objects.markers]
            min_dist_objs = sorted(obj_with_dists, key=lambda o:o[1])
        result =  []
        i = 0
        for obj in min_dist_objs[:8]:
            obj = obj[0]

            # img = CvBridge().imgmsg_to_cv2(self.rgb, 'bgr8')
            # obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
            # obj_size = [obj.scale.x,obj.scale.y,obj.scale.z]
            # u1,v1 = convert_point(obj_pos[0]-obj_size[0]/2,obj_pos[1]-obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
            # u2,v2 = convert_point(obj_pos[0]+obj_size[0]/2,obj_pos[1]+obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
            # img = img[int(0.95*v1):int(1.05*v2),int(0.95*u1):int(1.05*u2)]
            # cv2.imshow('img',img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            info = {}
            pos, quat = self.transform(obj.pose)
            info['euler_raw'] = tff.euler_from_quaternion([obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z, obj.pose.orientation.w])
            info['pos_raw'] = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
            info['euler'] = tff.euler_from_quaternion(quat)
            info['pos'] = pos
            info['scale'] = [obj.scale.x,obj.scale.y,obj.scale.z]
            info['grasp_orn'] = np.array([0, 0, 0, 0])
            info['name'] = 'cube_'+str(i)
            result.append(info)
            i += 1 # TODO (SURAJ):  Correct this 
        return result




def convert_point(x,y,z):
    u = x/z * intrinsics_RGB[0, 0] +  intrinsics_RGB[0, 2]
    v = y/z * intrinsics_RGB[1, 1] +  intrinsics_RGB[1, 2]
    return u,v

# rospy.init_node('her')
# receiver = GeneralPerception("stacking")
# while True:
#     receiver.wait_to_receive()
#     receiver.unregister()
#     img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
#     cv2.imshow('img',img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# print receiver.get_cubes()
# for i, obj in enumerate(receiver.get_cubes()):
#         print obj
#         obj = obj[0]
#         img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
#         cv2.imshow('img',img)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#         obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
#         obj_size = [obj.scale.x,obj.scale.y,obj.scale.z]
#         u1,v1 = convert_point(obj_pos[0]-obj_size[0]/2,obj_pos[1]-obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
#         u2,v2 = convert_point(obj_pos[0]+obj_size[0]/2,obj_pos[1]+obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
#         img = img[int(0.95*v1):int(1.05*v2),int(0.95*u1):int(1.05*u2)]
#         cv2.imshow('img',img)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()


# for i in range(5):
#     rospy.init_node('her')
#     receiver = GeneralPerception("stacking")
#     receiver.wait_to_receive()
#     receiver.unregister()
#     img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
#     cv2.imshow('img',img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     for i, obj in enumerate(receiver.objects.markers):
#         print obj
#         img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
#         obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
#         obj_size = [obj.scale.x,obj.scale.y,obj.scale.z]
#         u1,v1 = convert_point(obj_pos[0]-obj_size[0]/2,obj_pos[1]-obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
#         u2,v2 = convert_point(obj_pos[0]+obj_size[0]/2,obj_pos[1]+obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
#         img = img[int(0.95*v1):int(1.05*v2),int(0.95*u1):int(1.05*u2)]
#         cv2.imshow('img',img)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()

# obj = receiver.find_central_object()
# receiver.rgb.encoding = 'bgr8'
# img = CvBridge().imgmsg_to_cv2(receiver.rgb, 'bgr8')
# cv2.imshow('img',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# obj_pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
# obj_size = [obj.scale.x,obj.scale.y,obj.scale.z]
# u1,v1 = convert_point(obj_pos[0]-obj_size[0]/2,obj_pos[1]-obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
# u2,v2 = convert_point(obj_pos[0]+obj_size[0]/2,obj_pos[1]+obj_size[1]/2,obj_pos[2]-obj_size[2]/2)
# img = img[int(0.95*v1):int(1.05*v2),int(0.95*u1):int(1.05*u2)]
# cv2.imshow('img',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# receiver.depth.encoding = 'mono16'
# img = CvBridge().imgmsg_to_cv2(receiver.depth, 'mono16')
# img = img[int(0.95*v1):int(1.05*v2),int(0.95*u1):int(1.05*u2)]
# cv2.imshow('img',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()