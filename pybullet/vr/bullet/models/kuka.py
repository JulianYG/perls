import math
from bullet.models.core.robot import Robot
import pybullet as p

class Kuka(Robot):

	def __init__(self, pos, fixed=False, enableForceSensor=False):
		# Pos: Original y-coord for the robot arms  e.g., [0.3, -0.5]
		super(Kuka, self).__init__(enableForceSensor)
		self.FIX = fixed
		self.pos = pos
		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
		self.JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
		self.REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
		self.KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
		self.KUKA_GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, 
			-0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]
		self.THRESHOLD = 1.3
		self.MAX_FORCE = 500

	def _load_robot(self, ypos):

		# Gripper ID to arm ID
		for y_coord in ypos:
			self.arms.append(p.loadURDF('kuka_iiwa/model_vr_limits.urdf', 1.4, y_coord, 0.6, 0, 0, 0, 1))
			self.grippers.append(p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')[0])
		
		# Setup initial conditions for both arms
		for arm in self.arms:
			self._reset_robot(arm)

		# Setup initial conditions for both grippers
		for gripper in self.grippers:
			self._reset_robot_gripper(gripper)
			
		# Setup constraints on grippers
		for arm, gripper in zip(self.arms, self.grippers):
			self.constraints.append(p.createConstraint(arm, 6, gripper, 0, p.JOINT_FIXED, 
				[0,0,0], [0,0,0.05], [0,0,0], parentFrameOrientation=[0, 0, 0, 1]))


