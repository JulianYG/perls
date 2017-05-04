import numpy as np
from simulation.agents.robot import Robot

class Sawyer(Robot):

	def __init__(self, pos, fixed=False, enableForceSensor=False):
		self.nDOF = 7
		super(Sawyer, self).__init__(enableForceSensor)
		self.FIX = fixed
		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-3.05, -3.82, -3.05, -3.05, -2.98, -2.98, -4.71]
		self.UPPER_LIMITS = [3.05, 2.28, 3.05, 3.05, 2.98, 2.98, 4.71]
		self.JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]	
		self.REST_POSE = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
		self.MAX_FORCE = 500
		self.arm_urdf = 'sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf'
		self.positions = [[0.75, ypos, 0.625] for ypos in pos]

	def _roll_map(self):
		return lambda x: x

	def _pitch_map(self):
		return lambda x: x - np.pi / 2


