#!/usr/bin/env python

import redis
import sys, os
from os.path import join as pjoin

import rospy
import intera_interface
from ros_.robot import Robot
from ros_.utils.filter import Interpolator, UniformSubsampler, MovingAverageFilter
from ros_.utils.VRcontrol import RobotController
'''
Before initializing an instance of the Robot_Control class:
	rospy.init_node("sdk_wrapper") #initializes node on machine to talk to self.arm
	limb = intera_interface.Limb('right')
	gripper = intera_interface.Gripper('right')
	self.arm = Robot_Control(limb, gripper)
	
'''
sys.path.append(pjoin(os.getcwd(), 'bullet_'))
import pybullet as p
import Queue, time
import numpy as np
# event_queue = Queue.Queue(2048)
import time


from bullet_.simulation.robot import Sawyer
from bullet_.simulation.simulator import BulletSimulator

REST_POSE = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
LOWER_LIMITS = [-3.05, -3.82, -3.05, -3.05, -2.98, -2.98, -4.71]
UPPER_LIMITS = [3.05, 2.28, 3.05, 3.05, 2.98, 2.98, 4.71]
JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]	

p.connect(p.GUI)
sawyer = p.loadURDF('sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf',
	useFixedBase=1)
p.loadURDF('plane.urdf')

for jointIndex in range(7):
	p.resetJointState(sawyer, jointIndex, REST_POSE[jointIndex])

p.setRealTimeSimulation(1)
sim_initial_pos = p.getLinkState(sawyer, 6)[0]


rospy.init_node('sdk_wrapper')



# Initializing
rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
	'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}





class VR(object):

	def __init__(self, size=2048):
		self.vr_initial_pos = None
		

		self.cmd_queue = Queue.Queue(size)

		limb = intera_interface.Limb('right')
		gripper = intera_interface.Gripper('right')

		self.arm = Robot(limb, gripper)

		self.controller = RobotController(rate=100, command_queue=self.cmd_queue)

		
		self.r = redis.StrictRedis(host='localhost', port=6379, db=0)
		self.pubsub = self.r.pubsub()
		# sampler = UniformSubsampler(T=50)


	def turn_on(self):


		self.arm.set_init_positions(rest_pose)
		# Release the gripper first
		self.arm.slide_grasp(1)

		# Enqueue initial joint positions
		self.cmd_queue.put(rest_pose.values()[::-1])

		# Initialize reference frame positions
		self.arm_initial_pos = np.array(list(self.arm.get_tool_pose()[0]))


		self.prev_time = time.time()


	def start(self):

		
		self.pubsub.subscribe(**{'event_channel': self._event_handler})

		# Start thread
		# Using sleep_time=0.1 to update VR points on 10Hz
		self.client_thread = self.pubsub.run_in_thread(sleep_time=0.1)

		self.controller.control_loop()

	def _event_handler(self, msg):

		e = eval(msg['data'])
		pos, orn = e[1], e[2]

		if self.vr_initial_pos is None:
			self.vr_initial_pos = pos

		# control the gripper
		if e[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
			self.arm.slide_grasp(0)
		if e[6][33] & p.VR_BUTTON_WAS_RELEASED:
			self.arm.slide_grasp(1)
		if e[6][1] & p.VR_BUTTON_WAS_TRIGGERED:
			self.arm.set_init_positions(rest_pose)
			self.vr_initial_pos = np.array(list(e[1]))
			cmd_queue.put(rest_pose.values()[::-1])

		self.arm_rel_pos = self.arm.get_relative_pose()['position']
		rel_pose = np.array([self.arm_rel_pos.x ,self.arm_rel_pos.y, self.arm_rel_pos.z])

		# self.arm_target = self.arm_init + (vr_now - vr_init)
		rel_pos = np.array(pos) - np.array(self.vr_initial_pos)


		rel_pos = np.array([rel_pos[1], rel_pos[0], rel_pos[2]])

		self.arm_target_pos = self.arm_initial_pos + rel_pos

		# Use pybullet for now...
		sim_target_pos = sim_initial_pos + rel_pos

		t = time.time() - self.prev_time

		self.arm_joint_pos = p.calculateInverseKinematics(sawyer, 6, 
						sim_target_pos, (0, 1, 0, 0), lowerLimits=LOWER_LIMITS, 
						upperLimits=UPPER_LIMITS, jointRanges=JOINT_RANGE, 
						restPoses=REST_POSE, jointDamping=[0.1] * 7)

		for i in range(7):

			p.setJointMotorControl2(sawyer,
				i,
				p.POSITION_CONTROL,
				targetVelocity = 0,
				targetPosition=self.arm_joint_pos[i],
				force = 500, 
				positionGain=0.05,
				velocityGain = 1.)
		# if 0.1 < np.sqrt(np.sum(self.arm_target_pos ** 2)) < 0.8:

		if not self.cmd_queue.full():
			
			self.cmd_queue.put((self.arm_joint_pos, t))
		self.prev_time = time.time()


vr = VR()
vr.turn_on()

vr.start()
# while True:
# 	try:
# 		while not event_queue.empty():
# 			e = event_queue.get()
			
# 			# Get position and orientation
# 			# orn not used currently
			

# 			# subsample uniformly
# 			# if sampler.subsample(pos) is None:
# 			# 	continue

			
# 				# if not self.arm.reach_absolute({'position': self.arm_target_pos, 'orientation': (0, 1, 0, 0)}):
# 				# 	pos2 = self.arm_initial_pos

# 			#TODO: do the interp and minjerk with pos1 and pos2  

# 			# Update pos1 now
# 			# pos1 = pos2

# 	except KeyboardInterrupt:
# 		client_thread.stop()
# 		pubsub.unsubscribe()
# 		self.arm.shutdown()
# 		sys.exit(0)

