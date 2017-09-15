#!/usr/bin/env python

"""
Execute a grasp from DexNet
"""
import argparse
import logging
import IPython
import numpy as np
import os
import sys
import time
import rospy
import random
from autolab_core import RigidTransform, YamlConfig
from perception import RgbdImage, RgbdSensorFactory

from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
from gqcnn import Visualizer as vis
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,TransformStamped
from std_msgs.msg import ColorRGBA
import tf
sys.path.append(os.path.abspath('../'))
from sawyer import SawyerArm
def show_endpoints(position,quat,frame='/base'):
    rate = rospy.Rate(1)

    iterations = 0
    while not rospy.is_shutdown() and iterations <= 1:
        pub = rospy.Publisher('object_grasps', Marker, queue_size = 10)
        marker = Marker()
        marker.header.frame_id = '/base'

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.id = 0
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.z = 0.025
        marker.scale.x = 0.05
        marker.scale.y = 0.175
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        marker.color.a = 1.0

        pub.publish(marker)
        iterations+=1
        rate.sleep()
        #marker.type = marker.LINE_LIST
        #marker.id = 1
        #marker.scale.x = 0.001
        #marker.color.a = 1.0
        #marker.colors = [colors[i] for i in range(0,len(colors),2)]

        #pub.publish(marker)

if __name__ == '__main__':
    # set up logger
    rospy.init_node("grasp_display")
    logging.getLogger().setLevel(logging.DEBUG)

    # parse args
    parser = argparse.ArgumentParser(description='Capture a set of test images from the Kinect2')
    parser.add_argument('--config_filename',
                        type=str,
                        default='/home/cvgl_ros/deps/dexnet/cfgs/policy.yaml',
                        help='path to configuration file to use')
    args = parser.parse_args()
    config_filename = args.config_filename

    # read config
    config = YamlConfig(config_filename)
    sensor_type = config['sensor']['type']
    sensor_frame = config['sensor']['frame']
    inpaint_rescale_factor = config['inpaint_rescale_factor']
    policy_config = config['policy']

    # read camera calib
    tf_filename = '%s_to_world.tf' %(sensor_frame)
    T_camera_world = RigidTransform.load(os.path.join(config['calib_dir'], sensor_frame, tf_filename))

    # setup sensor
    sensor = RgbdSensorFactory.sensor(sensor_type, config['sensor'])
    sensor.start()
    camera_intr = sensor.ir_intrinsics

    # read images
    color_im, depth_im, _ = sensor.frames()
    color_im = color_im.inpaint(rescale_factor=inpaint_rescale_factor)
    depth_im = depth_im.inpaint(rescale_factor=inpaint_rescale_factor)
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
    state = RgbdImageState(rgbd_im, camera_intr)

    # init policy
    policy = CrossEntropyAntipodalGraspingPolicy(policy_config)
    policy_start = time.time()
    action = policy(state)
    logging.info('Planning took %.3f sec' %(time.time() - policy_start))

    if policy_config['vis']['final_grasp']:
        vis.figure(size=(10,10))
        vis.subplot(1,2,1)
        vis.imshow(rgbd_im.color)
        vis.grasp(action.grasp, scale=1.5, show_center=False, show_axis=True)
        vis.title('Planned grasp on color (Q=%.3f)' %(action.q_value))
        vis.subplot(1,2,2)
        vis.imshow(rgbd_im.depth)
        vis.grasp(action.grasp, scale=1.5, show_center=False, show_axis=True)
        vis.title('Planned grasp on depth (Q=%.3f)' %(action.q_value))
        vis.show()
    pose = action.grasp.pose()
    sawyer = SawyerArm(motion_planning=False)
    t = tf.Transformer(True, rospy.Duration(10.0))
    m = TransformStamped()
    m.header.frame_id = "/grasp"
    m.child_frame_id = "/ee"
    m.transform.translation.x = 0.055
    m.transform.translation.y = 0
    m.transform.translation.z = 0
    m.transform.rotation.w = 0.68#0.707
    m.transform.rotation.x = -0.25#0.707
    m.transform.rotation.y = 0#0.707
    m.transform.rotation.z = -0.68 #.707#-0.707
    t.setTransform(m)
    m = TransformStamped()
    m.header.frame_id = "/kinect2_ir_optical_frame"
    m.child_frame_id = "/grasp"
    m.transform.translation.x = pose.position[0]
    m.transform.translation.y = pose.position[1]
    m.transform.translation.z = pose.position[2]
    m.transform.rotation.w = pose.quaternion[0]
    m.transform.rotation.x = pose.quaternion[1]
    m.transform.rotation.y = pose.quaternion[2]
    m.transform.rotation.z = pose.quaternion[3]
    t.setTransform(m)
    m = TransformStamped()
    m.header.frame_id = "/base"
    m.child_frame_id = "/kinect2_ir_optical_frame"
    m.transform.translation.x = 0.6901
    m.transform.translation.y = 0.105893
    m.transform.translation.z = 1.00102
    m.transform.rotation.w = 0
    m.transform.rotation.x = 0.707
    m.transform.rotation.y = 0.707
    m.transform.rotation.z = 0
    t.setTransform(m)
    pos, quat = t.lookupTransform("/base", "/ee", rospy.Time())
    print(quat)
    show_endpoints(pos,quat)
    #sawyer.move_to_with_grasp(pos[0],pos[1],pos[2]+0.125,0.1,0.05,quat)
