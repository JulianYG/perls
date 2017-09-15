#!/usr/bin/env python
import redis
import sys, os
from os.path import join as pjoin

path = os.path.dirname(os.path.abspath(__file__))
rpath = os.path.normpath(pjoin(path, '..'))
sys.path.append(pjoin(path, '../perls'))

import rospy
import intera_interface

from ros_.robot import Robot
from ros_.utils.filter import Interpolator, UniformSubsampler, MovingAverageFilter
from ros_.utils.VRcontrol import RobotController

import pybullet as p
import Queue, time
import numpy as np

from bullet_.simulation.arm import Sawyer
from bullet_.simulation.simulator import BulletSimulator

REST_POSE = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
LOWER_LIMITS = [-3.05, -3.82, -3.05, -3.05, -2.98, -2.98, -4.71]
UPPER_LIMITS = [3.05, 2.28, 3.05, 3.05, 2.98, 2.98, 4.71]
JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]	

FIXED = False

p.connect(p.GUI)
sawyer = p.loadURDF('sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf',
	(0, 0, 0.9), useFixedBase=1)
p.loadURDF('plane.urdf')

for jointIndex in range(7):
	p.resetJointState(sawyer, jointIndex, REST_POSE[jointIndex])

p.setRealTimeSimulation(1)

rospy.init_node('server')

# Initializing
rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
	'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}


class VRSawyer(object):

	def __init__(self):
		self.vr_initial_pos = None

		limb = intera_interface.Limb('right')
		gripper = intera_interface.Gripper('right')

		self.arm = Robot(limb, gripper)
		self.controller = RobotController(rate=100)

		self.r = redis.StrictRedis(host='localhost', port=6379, db=0)
		self.pubsub = self.r.pubsub()
		self.sim_initial_pos = p.getLinkState(sawyer, 6)[0]
		self.controller_initial_orn = None

		self.engaged = False

	def turn_on(self):

		self.arm.set_init_positions(rest_pose)
		# Release the gripper first
		self.arm.slide_grasp(1)

		# Initialize reference frame positions
		self.arm_initial_pos = np.array(list(self.arm.get_tool_pose()[0]))

	def start(self):

		self.pubsub.subscribe(**{'event_channel': self._event_handler})

		# Start thread
		# Using sleep_time=0.1 to hold VR points on 10Hz
		self.client_thread = self.pubsub.run_in_thread(sleep_time=0.1)

		self.prev_time = time.time()

		while True:
			self.controller.control_loop()

	@property
	def gripper_orn(self):
		return np.array(self.joint_positions[-2:])

	@property
	def joint_positions(self):
		return np.array(self.arm.get_joint_angles().values()[::-1])

	def gripper_controller(self, event):

		# control the gripper
		if event[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
			self.arm.slide_grasp(0)
		if event[6][33] & p.VR_BUTTON_WAS_RELEASED:
			self.arm.slide_grasp(1)
		if event[6][1] & p.VR_BUTTON_WAS_TRIGGERED:
			self.arm.set_init_positions(rest_pose)
			self.vr_initial_pos = np.array(list(event[1]))
			self.controller.put_item(rest_pose.values()[::-1])

		# Control arm engage
		if event[6][32] & p.KEY_WAS_TRIGGERED:
			self.vr_initial_pos = event[1]
			self.engaged = True
			self.controller.reset()

		if event[6][32] & p.VR_BUTTON_WAS_RELEASED:
			self.engaged = False

	def _event_handler(self, msg):

		e = eval(msg['data'])
		pos, orn = e[1], e[2]
		
		self.gripper_controller(e)

		if self.engaged:

			rel_pos = np.array(pos) - np.array(self.vr_initial_pos)
			
			if np.sum(rel_pos ** 2) >= 2.0:
				print(rel_pos, 'Out of range!')
				return

			self.arm_target_pos = self.arm_initial_pos + rel_pos

			# Use pybullet for now...
			sim_target_pos = self.sim_initial_pos + rel_pos * 2

			t = time.time() - self.prev_time

			# Get current gripper orientation
			gripper_orn = p.getLinkState(sawyer, 6)[1]

			arm_joint_pos = p.calculateInverseKinematics(sawyer, 6, 
				sim_target_pos, gripper_orn, lowerLimits=LOWER_LIMITS, 
				upperLimits=UPPER_LIMITS, jointRanges=JOINT_RANGE, 
				restPoses=REST_POSE, jointDamping=[0.1] * 7)

			for i in range(7):

				p.setJointMotorControl2(sawyer,
					i,
					p.POSITION_CONTROL,
					targetVelocity=0,
					targetPosition=arm_joint_pos[i],
					force=500, 
					positionGain=0.05,
					velocityGain=1.)

			self.controller.put_item((arm_joint_pos, t))
			self.controller_initial_orn = orn

		else:
			if not FIXED and self.controller_initial_orn:
				# Choose to only move relatively
				x, y, _ = p.getEulerFromQuaternion(orn)
				x_ini, y_ini, _ = p.getEulerFromQuaternion(self.controller_initial_orn)

				gripper_rel_orn = np.array([x, y]) - np.array([x_ini, y_ini])

				# x for joint 6, y for joint 5
				gripper_target_orn = np.clip(self.gripper_orn + gripper_rel_orn[::-1], 
					LOWER_LIMITS[5:7], UPPER_LIMITS[5:7])

				t = time.time() - self.prev_time
				jpos = self.joint_positions

				# Needs scaling
				jpos[5] = gripper_target_orn[0] / 2
				jpos[6] = gripper_target_orn[1] / 2				

				self.controller.put_item((jpos, t))

			self.sim_initial_pos = p.getLinkState(sawyer, 6)[0]
			self.arm_initial_pos = np.array(list(self.arm.get_tool_pose()[0]))

			jpos = self.joint_positions

			# Sync the current robot state with simulation state
			for i in range(7):
				p.setJointMotorControl2(sawyer,
					i,
					p.POSITION_CONTROL,
					targetVelocity = 0,
					targetPosition=jpos[i],
					force=500, 
					positionGain=0.5,
					velocityGain = 1.)
				p.resetJointState(sawyer, i, 
					jpos[i], 0)

		self.prev_time = time.time()

def run():
	vr = VRSawyer()
	vr.turn_on()

	vr.start()

if __name__ == '__main__':
	run()




