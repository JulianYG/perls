#!/usr/bin/env python

"""
A tracker that calibrates by tracking the robot gripper 
"""

import numpy as np

import sys, os
from os.path import join as pjoin

import rospy
import rosparam
import intera_interface
import time
import pybullet as p
sys.path.append(os.path.abspath(os.path.join(__file__, '../../')))
from robot import Robot


if rospy.get_name() == '/unnamed':
    rospy.init_node('kinect_tracker')

limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, None)
# [0.5, 0.2, -0.0251 + 0.3]

np.set_printoptions(formatter={'float': lambda x: "{0:0.8f}".format(x)})
end_state = dict(position=robot.get_tool_pose()[0],
                         orientation=[0, 0, 1, 0])
            # Move to target position
robot.reach_absolute(end_state)
# gripper_pos = np.array(robot.get_tool_pose()[0], dtype=np.float32)
# print((robot.get_tool_pose()[0][-1] + 0.0251) * 1000)

print(np.array(robot.get_tool_pose()[1], dtype=np.float32))
print(p.getEulerFromQuaternion(np.array(robot.get_tool_pose()[1], dtype=np.float32)))
print(robot.get_joint_angles())
# gripper_pos = np.array(robot.get_tool_pose()[0], dtype=np.float32)
# print(gripper_pos)
# gripper_ori = np.array(robot.get_tool_pose()[1], dtype=np.float32)
# trans_matrix = np.array(p.getMatrixFromQuaternion(gripper_ori)).reshape(3,3)
# pos = gripper_pos + trans_matrix[-1] * 0.02534
# print(pos)
# 0.27188136, 0.27217033, 0.27493553, 0.2706659 , 0.27151584, 0.26897513
# mean: 0.27169068166666671
# ground truth 0.29783455
# gap: 0.026143868333333264

# 0.22716515, 0.22553267, 0.22491231, 0.22534641, 0.22611915, 0.22591888, 0.2275707 , 0.22545462, 0.22634964, 0.22695564, 0.22560154
# mean: 0.22608424636363633
# ground truth: 0.25203449
# gap: 0.025950243636363668


# 0.27545049, 0.27433034, 0.27341267, 0.27277633, 0.27599709, 0.2750129 , 0.27555113, 0.27515336, 0.27544927, 0.27610829, 0.27330648, 0.27460005, 0.27257147, 0.27389752, 0.27269523, 0.27132433, 0.27376994
# mean: 0.27420040529411766
# ground truth: 0.29931954
# gap: 0.02511913470588234