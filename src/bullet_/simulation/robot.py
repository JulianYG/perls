import numpy as np
import sys, os
from os.path import join as pjoin
sys.path.append(os.path.abspath(pjoin(os.path.dirname(__file__))))

from agent import Robot

class Sawyer(Robot):

	def __init__(self, pos, fixed=False, enableForceSensor=False):
		self.nDOF = 7
		super(Sawyer, self).__init__(enableForceSensor,
			gripper_urdf='rethink_ee_description/urdf/right_end_effector.urdf')
		self.FIX = fixed
		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-3.05, -3.82, -3.05, -3.05, -2.98, -2.98, -4.71]
		self.UPPER_LIMITS = [3.05, 2.28, 3.05, 3.05, 2.98, 2.98, 4.71]
		self.JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]	
		self.REST_POSE = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
		self.MAX_FORCE = 500
		self.arm_urdf = 'sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf'
		self.positions = [[0.45, ypos, 0.9] for ypos in pos]

		self.GRIPPER_REST_POS = [0., 0.020833, 0., -0.020833, 0.]
		self.GRIPPER_CLOZ_POS = [0., 0., 0., 0., 0.]

	def _roll_map(self):
		return lambda x: x

	def _pitch_map(self):
		return lambda x: x - np.pi / 4


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





