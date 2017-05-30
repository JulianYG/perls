#!/usr/bin/env python
import redis
import sys, os, subprocess
from os.path import join as pjoin

import rospy
import intera_interface
from ros_.robot import Robot
from ros_.utils import Interpolator, UniformSubsampler, MovingAverageFilter

'''
Before initializing an instance of the Robot_Control class:
	rospy.init_node("sdk_wrapper") #initializes node on machine to talk to arm
	limb = intera_interface.Limb('right')
	gripper = intera_interface.Gripper('right')
	arm = Robot_Control(limb, gripper)
	
'''
rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

import pybullet as p
import Queue, time
import numpy as np
event_queue = Queue.Queue(4096)

def _event_handler(msg):
	if not event_queue.full():
		event_queue.put(eval(msg['data']))

r = redis.StrictRedis(host='localhost', port=6379, db=0)
pubsub = r.pubsub()
pubsub.subscribe(**{'event_channel': _event_handler})

# Start thread
# Using sleep_time=0.1 to update VR points on 10Hz
client_thread = pubsub.run_in_thread(sleep_time=0.1)

arm = Robot(limb, gripper)

# Initializing
rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
	'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}
arm.set_init_positions(rest_pose)
# Release the gripper first
arm.slide_grasp(1)

# Initialize reference frame positions
arm_initial_pos = np.array(list(arm.get_tool_pose()[0]))
vr_initial_pos = None

# Initialize initial pos1 for interpolation
pos1 = arm_initial_pos

while True:
	if not event_queue.empty():
		vr_initial_pos = np.array(list(event_queue.get()[1]))
		break

sampler = UniformSubsampler(T=50)

while True:
	try:
		while not event_queue.empty():
			e = event_queue.get()
			
			# Get position and orientation
			# orn not used currently
			pos, orn = e[1], e[2]

			# control the gripper
			if e[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
				arm.slide_grasp(0)
			if e[6][33] & p.VR_BUTTON_WAS_RELEASED:
				arm.slide_grasp(1)
			if e[6][1] & p.VR_BUTTON_WAS_TRIGGERED:
				arm.set_init_positions(rest_pose)
				vr_initial_pos = np.array(list(e[1]))

			# subsample uniformly
			# if sampler.subsample(pos) is None:
			# 	continue

			arm_rel_pos = arm.get_relative_pose()['position']
			rel_pose = np.array([arm_rel_pos.x ,arm_rel_pos.y, arm_rel_pos.z])

			vr_rel_pos = np.array(list(pos2) - vr_initial_pos)
			pos2 = arm_initial_pos + vr_rel_pos

			if 0.1 < np.sqrt(np.sum(pos2 ** 2)) < 0.8:
				if not arm.reach_absolute({'position': target_pos, 'orientation': (0, 1, 0, 0)}):
					pos2 = arm_initial_pos

			#TODO: do the interp and minjerk with pos1 and pos2  

			# Update pos1 now
			pos1 = pos2

	except KeyboardInterrupt:
		client_thread.stop()
		pubsub.unsubscribe()
		arm.shutdown()
		sys.exit(0)

