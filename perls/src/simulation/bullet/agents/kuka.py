import numpy as np
from bullet.agents.robot import *

class Kuka(Robot):

	def __init__(self, pos, enableForceSensor=False, fixed=False):
		# Pos: Original y-coord for the robot arms  e.g., [0.3, -0.5]
		self.nDOF = 7
		super(Kuka, self).__init__(enableForceSensor)
		self.FIX = fixed
		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, np.pi / 2, 0, -np.pi * 0.66, 0]

		# Gripper positions are wsg50, defined in robot.py
		# Common threshold is 1.3 meters;
		# Default joint damping is 0.1 for all joints
		self.MAX_FORCE = 500
		self.arm_urdf = 'kuka_iiwa/model_vr_limits.urdf'
		self.positions = [[1.4, ypos, 0.6] for ypos in pos]

	def _roll_map(self):
		return lambda x: x

	def _pitch_map(self):
		return lambda x: -x


