import math
from bullet.models.robot import Robot
import pybullet as p

class Kuka(Robot):

	def __init__(self, pos, fixed=False, enableForceSensor=False):
		# Pos: Original y-coord for the robot arms  e.g., [0.3, -0.5]
		super(Kuka, self).__init__(pos, enableForceSensor)
		self.FIX = fixed
		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
		self.JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
		self.GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
		self.GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, 
			-0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]
		self.THRESHOLD = 1.3
		self.MAX_FORCE = 500

	def setup_scene(self, task):
		"""
		Basic scene needed for running tasks
		"""
		self.load_basic_env()
		self.side_obj_cnt = p.getNumBodies()
		self._load_task(task)
		self.env_obj = range(p.getNumBodies(), self.side_obj_cnt)
		p.setGravity(0, 0, -9.81)

	def reach(self, arm_id, eef_pos, eef_orien, fixed):

		if fixed:
			joint_pos = p.calculateInverseKinematics(arm_id, 6, eef_pos, eef_orien, 
				lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, 
				jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			for i in range(len(joint_pos)):
				p.setJointMotorControl2(arm_id, i, p.POSITION_CONTROL, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=self.MAX_FORCE)
		else:
			joint_pos = p.calculateInverseKinematics(arm_id, 6, eef_pos, 
				lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, 
				jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			# Only need links 1- 5, no need for joint 4-6 with pure position IK
			for i in range(len(joint_pos) - 3):
				p.setJointMotorControl2(arm_id, i, p.POSITION_CONTROL, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=self.MAX_FORCE)
			
			x, y, z = p.getEulerFromQuaternion(eef_orien)

			#TO-DO: add wait till fit

			# Link 4 needs protection
			if self.LOWER_LIMITS[6] < x < self.UPPER_LIMITS[6]:	# JOInt limits!!
				p.setJointMotorControl2(arm_id, 6, p.POSITION_CONTROL, 
					targetPosition=x, targetVelocity=0, positionGain=0.02, velocityGain=1, force=self.MAX_FORCE)
			else:
				p.addUserDebugText('Warning: you are flipping arm link 6', p.getLinkState(arm_id, 0)[0], 
					textColorRGB=(255, 0, 0), lifeTime=1.5)
				p.setJointMotorControl2(arm_id, 6, p.POSITION_CONTROL, 
					targetPosition=joint_pos[6], targetVelocity=0, positionGain=0.01, velocityGain=1.0, force=self.MAX_FORCE)

			if self.LOWER_LIMITS[5] < y < self.UPPER_LIMITS[5]:
				p.setJointMotorControl2(arm_id, 5, p.POSITION_CONTROL, 
					targetPosition=-y, targetVelocity=0, positionGain=0.03, velocityGain=1.0, force=self.MAX_FORCE)
			else:
				p.addUserDebugText('Warning: you are flipping arm link 5', p.getLinkState(arm_id, 1)[0], 
					textColorRGB=(255, 0, 0), lifeTime=1.5)
				p.setJointMotorControl2(arm_id, 5, p.POSITION_CONTROL, 
					targetPosition=joint_pos[5], targetVelocity=0, positionGain=0.01, velocityGain=1.0, force=self.MAX_FORCE)

	def _load_tools(self, ypos):

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

