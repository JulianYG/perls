#!/usr/bin/env python
# Intera SDK Wrapper for use w/ PyBullet

import rospy
import sys, os
from os.paht import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'ros_'))


'''
Before initializing an instance of the Robot_Control class:
	rospy.init_node("sdk_wrapper") #initializes node on machine to talk to arm
	limb = intera_interface.Limb('right')
	gripper = intera_interface.Gripper('right')
	arm = Robot_Control(limb, gripper)
	
'''
rospy.init_node('sdk_wrapper')


