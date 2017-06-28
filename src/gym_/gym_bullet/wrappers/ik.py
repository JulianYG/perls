import numpy as np
import os, sys
from os.path import join as pjoin
from .base import Wrapper

class VelocityControl(Wrapper):

	def __init__(self, agent):

		super(VelocityControl, self).__init__(agent)

	def reset(self, scene, task, gui):
		"""
		:return initial state
		"""
		self.agent.build(scene, task, gui, reset=True)
		return self.states

	# Make use of self.agent
	def step(self, action):	
		"""
		User defined step_helper. Takes pybullet tool agent and action,
		returns tuple of observation, reward, done status, and info.
		A high level pybullet wrapper for the gym step function

		:return observation, reward, done, info
		""" 
		arm = self.agent.get_tool_ids()[0]
			
		# 0 for velocity control
		self.agent.set_tool_joint_states(arm, action, ctrl=0)
		
		state = self.states
		reached = False
		if np.sqrt(np.sum((np.array(state[0])) ** 2)) < 2e-3:
			reached = True

		return state, 1., reached, {}


class TorqueControl(Wrapper):

	def __init__(self, agent):

		super(TorqueControl, self).__init__(agent)

	def reset(self, scene, task, gui):
		"""
		:return initial state
		"""

		# Reset false to enable torque control 
		self.agent.build(scene, task, gui, reset=False)
		return self.states

	def step(self, action):	
		"""
		User defined step_helper. Takes pybullet tool agent and action,
		returns tuple of observation, reward, done status, and info.
		A high level pybullet wrapper for the gym step function

		:return observation, reward, done, info
		""" 
		arm = self.agent.get_tool_ids()[0]

		# 1 stands for torque control
		self.agent.set_tool_joint_states(arm, action, ctrl=1)

		state = self.states
		reached = False
		if np.sqrt(np.sum((np.array(state[0])) ** 2)) < 2e-4:
			reached = True

		return state, .1, reached, {}


class PositionControl(Wrapper):

	def __init__(self, agent):

		super(PositionControl, self).__init__(agent)

	def reset(self, scene, task, gui):
		"""
		:return initial state
		"""
		self.agent.build(scene, task, gui, reset=True)
		return self.states

	def step(self, action):	
		"""
		User defined step_helper. Takes pybullet tool agent and action,
		returns tuple of observation, reward, done status, and info.
		A high level pybullet wrapper for the gym step function

		:return observation, reward, done, info
		""" 
		arm = self.agent.get_tool_ids()[0]
	
		# Cheating to set correct joint positions without pi(s) = a		
		goal = np.array([0.8, 0., 1.0])
		import pybullet as p
		cheat_action = p.calculateInverseKinematics(arm, 6,
			goal, (0, 1, 0, 0), lowerLimits=self.agent.LOWER_LIMITS,
			upperLimits=self.agent.UPPER_LIMITS, jointRanges=self.agent.JOINT_RANGE,
			restPoses=self.agent.REST_POSE, jointDamping=self.agent.JOINT_DAMP)

		# 2 for position control
		# Note position gain high to push for accuracy
		self.agent.set_tool_joint_states(arm, cheat_action, ctrl=2, positionGain=0.5)

		state = self.states
		reached = False
		if np.sqrt(np.sum((np.array(state[0])) ** 2)) < 2e-4:
			reached = True

		return state, 1., reached, {}

