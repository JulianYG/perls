__package__ = 'bullet_.simulation'

import numpy as np
import pybullet as p

from .utils import handler
from .utils.misc import Constant

from .tool import Tool

class Arm(Tool):
	def __init__(self, enableForceSensor,
				 gripper_file='gripper/wsg50_one_motor_gripper_new_free_base.sdf'):

		super(Arm, self).__init__(enableForceSensor)
		self.THRESHOLD = 1.3

		# Common type of grippers for all robots
		self.gripper_file = gripper_file
		self.GRIPPER_REST_POS = [0., -0.011130, -0.206421, 
								 0.205143, -0.009999, 0., -0.010055, 0.]
		self.GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611,
								 -0.7479294372303137, 0.05054599996976922, 0.0, 
								 0.049838105678835724, 0.0]
		self.JOINT_DAMP = [.1] * self.nDOF
		self.ee_offset = 0.05

	def get_tool_ids(self):
		return self.arms

	def get_tool_pose(self, tool_id):
		# End effector pose for robot
		# Returns linkWorldPos, linkWorldOrn
		return self.get_tool_link_state((tool_id, -1))

	def control(self, event, ctrl_map):

		ctrl_id = event[0]
		arm_id = ctrl_map[Constant.ARM][ctrl_id]
		gripper_id = ctrl_map[Constant.GRIPPER][ctrl_id]
		self._set_camera(gripper_id)
		self.slide_grasp(gripper_id, event)

		# Allows robot arm control by VR controllers
		if self.get_tool_control_deviation(arm_id, event[1]) < self.THRESHOLD:
			self._engage(arm_id, event)
			return 0
		else:
			self._disengage(arm_id, event)
			return -1

	def grip(self, gripper):
		for i in range(p.getNumJoints(gripper)):
			p.setJointMotorControl2(gripper, i, Constant.POS_CTRL,
									targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

	def release(self, gripper):
		for i in range(p.getNumJoints(gripper)):
			p.setJointMotorControl2(gripper, i, Constant.POS_CTRL,
									targetPosition=self.GRIPPER_REST_POS[i], force=50)

	def slide_grasp(self, gripper, event):

		# TODO: Add slider for the grippers
		analog = event[3]
		if event[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, Constant.POS_CTRL,
										targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

		if event[6][33] & p.VR_BUTTON_WAS_RELEASED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, Constant.POS_CTRL,
										targetPosition=self.GRIPPER_REST_POS[i], force=50)

	def reach(self, arm_id, eef_pos, eef_orien, fixed, ctrl=Constant.POS_CTRL,
			  null_space=True, expedite=False):
		"""
		Lowest level of implementation for faster operation
		"""
		pos_gain = 0.05
		if expedite:
			pos_gain = 0.5
		if fixed:
			if null_space:
				joint_pos = p.calculateInverseKinematics(arm_id, self.nDOF - 1,
														 eef_pos, eef_orien, lowerLimits=self.LOWER_LIMITS,
														 upperLimits=self.UPPER_LIMITS, jointRanges=self.JOINT_RANGE,
														 restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			else:
				joint_pos = p.calculateInverseKinematics(arm_id, self.nDOF - 1,
														 eef_pos, eef_orien)
			for i in range(len(joint_pos)):
				p.setJointMotorControl2(arm_id, i, ctrl, targetPosition=joint_pos[i],
										targetVelocity=0, positionGain=pos_gain,
										velocityGain=1.0, force=self.MAX_FORCE)
		else:
			if null_space:
				joint_pos = p.calculateInverseKinematics(arm_id, self.nDOF - 1,
														 eef_pos, lowerLimits=self.LOWER_LIMITS,
														 upperLimits=self.UPPER_LIMITS, jointRanges=self.JOINT_RANGE,
														 restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			else:
				joint_pos = p.calculateInverseKinematics(arm_id, self.nDOF - 1,
														 eef_pos)
			if eef_orien == None:
				for i in range(len(joint_pos)):
					p.setJointMotorControl2(arm_id, i, ctrl,
											targetPosition=joint_pos[i], targetVelocity=0,
											positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)
				return joint_pos

			# Only need links 1- 5, no need for joint 4-6 with pure position IK
			for i in range(len(joint_pos) - 3):
				p.setJointMotorControl2(arm_id, i, ctrl,
										targetPosition=joint_pos[i], targetVelocity=0, positionGain=pos_gain,
										velocityGain=1.0, force=self.MAX_FORCE)

			x, y, _ = p.getEulerFromQuaternion(eef_orien)

			# TO-DO: add wait till fit

			# Link 6 needs protection
			if self.LOWER_LIMITS[self.nDOF - 1] < x < self.UPPER_LIMITS[self.nDOF - 1]:
				p.setJointMotorControl2(arm_id, self.nDOF - 1,
										ctrl,
										targetPosition=self._roll_map()(x), targetVelocity=0,
										positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)
			else:
				p.setJointMotorControl2(arm_id, self.nDOF - 1,
										ctrl,
										targetPosition=joint_pos[self.nDOF - 1], targetVelocity=0,
										positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)

				self.mark('Warning: you are flipping arm link {}'.format(self.nDOF - 1),
						  p.getLinkState(arm_id, 0)[0], time=1.5)
				raise handler.IllegalOperation(self.nDOF - 1)

			if self.LOWER_LIMITS[self.nDOF - 2] < y < self.UPPER_LIMITS[self.nDOF - 2]:
				p.setJointMotorControl2(arm_id, self.nDOF - 2, ctrl,
										targetPosition=self._pitch_map()(y), targetVelocity=0,
										positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)
			else:
				p.setJointMotorControl2(arm_id, self.nDOF - 2, ctrl,
										targetPosition=joint_pos[self.nDOF - 2], targetVelocity=0,
										positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)

				self.mark('Warning: you are flipping arm link {}'.format(self.nDOF - 2),
						  p.getLinkState(arm_id, 1)[0], time=1.5)
				raise handler.IllegalOperation(self.nDOF - 2)
		return joint_pos

	def _load_tools(self, positions):
		# Gripper ID to arm ID
		for i in range(len(positions)):

			pos = positions[i]
			arm_id = p.loadURDF(self.arm_urdf,
								pos, [0, 0, 0, 1], useFixedBase=True)
			self.arms.append(arm_id)

			df_format = self.gripper_file.split('.')[1]

			if df_format == 'sdf':
				gripper_id = p.loadSDF(self.gripper_file)[0]
			else:
				gripper_id = p.loadURDF(self.gripper_file, (0, 0, 0.9))
			self.grippers.append(gripper_id)

			self.name_dic[arm_id] = '{}_{}'.format(p.getBodyInfo(arm_id)[1], i)
			self.name_dic[gripper_id] = '{}_{}'.format(p.getBodyInfo(gripper_id)[1], i)

		# Setup initial conditions for both arms
		for arm in self.arms:
			self._reset_robot(arm)

		# Setup initial conditions for both grippers
		for gripper in self.grippers:
			self._reset_robot_gripper(gripper)

		# Setup constraints on grippers
		for arm, gripper in zip(self.arms, self.grippers):
			self.constraints.append(p.createConstraint(arm, self.nDOF - 1,
													   # gripper, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0],
													   gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, self.ee_offset],
													   [0, 0, 0],
													   parentFrameOrientation=[0, 0, 0, 1],
													   childFrameOrientation=[0., 0, 0.707, 0.707]))
		self.solo = len(self.arms) == 1

	def _engage(self, robot, controller_event):
		controller_pos = controller_event[1]
		controller_orn = controller_event[2]
		targetPos = controller_pos
		eef_orn = controller_orn
		if controller_event[6][32] & p.VR_BUTTON_IS_DOWN:
			if self.FIX:
				self.reach(robot, targetPos, (0, 1, 0, 0), self.FIX)
			else:
				self.reach(robot, targetPos, controller_orn, self.FIX)

	def _disengage(self, robot, controller_event):

		if controller_event[6][32] & p.VR_BUTTON_IS_DOWN:
			for jointIndex in range(p.getNumJoints(robot)):
				p.setJointMotorControl2(robot, jointIndex, Constant.POS_CTRL,
										targetPosition=self.REST_POSE[jointIndex],
										targetVelocity=0, positionGain=0.03, velocityGain=1,
										force=self.MAX_FORCE)

	def _reset_robot(self, robot):
		for jointIndex in range(p.getNumJoints(robot)):
			# p.resetJointState(robot, jointIndex, self.REST_POSE[jointIndex])
			p.setJointMotorControl2(robot, jointIndex, Constant.POS_CTRL,
									self.REST_POSE[jointIndex], 0)

	def _reset_robot_gripper(self, robot_gripper):
		for jointIndex in range(p.getNumJoints(robot_gripper)):
			# p.resetJointState(robot_gripper, jointIndex, self.GRIPPER_REST_POS[jointIndex])
			p.setJointMotorControl2(robot_gripper, jointIndex,
									Constant.POS_CTRL, self.GRIPPER_REST_POS[jointIndex], 0)


class Sawyer(Arm):

	def __init__(self, pos, fixed=False, enableForceSensor=False):
		self.nDOF = 7
		super(Sawyer, self).__init__(enableForceSensor,
			gripper_file='rethink_ee_description/urdf/right_end_effector.urdf')
		self.FIX = fixed

		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-3.05, -3.82, -3.05, -3.05, -2.98, -2.98, -4.71]
		self.UPPER_LIMITS = [3.05, 2.28, 3.05, 3.05, 2.98, 2.98, 4.71]
		self.JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]	
		self.REST_POSE = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
		self.MAX_FORCE = 500
		self.arm_urdf = 'sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf'
		self.positions = [[0.45, ypos, 0.8] for ypos in pos]

		self.GRIPPER_REST_POS = [0., 0.020833, 0., -0.020833, 0.]
		self.GRIPPER_CLOZ_POS = [0., 0., 0., 0., 0.]
		self.ee_offset = 0.195

	def _roll_map(self):
		return lambda x: x

	def _pitch_map(self):
		return lambda x: x - np.pi / 4

	def _set_camera(self, uid):
		p.resetDebugVisualizerCamera(0.4, 30, -120, 
			p.getBasePositionAndOrientation(uid)[0])


class Kuka(Arm):

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

		# Gripper positions are wsg50, defined in arm.py
		# Common threshold is 1.3 meters;
		# Default joint damping is 0.1 for all joints
		self.MAX_FORCE = 500
		self.arm_urdf = 'kuka_iiwa/model_vr_limits.urdf'
		self.positions = [[1.4, ypos, 0.6] for ypos in pos]

	def _roll_map(self):
		return lambda x: x

	def _pitch_map(self):
		return lambda x: -x

	def _set_camera(self, uid):
		p.resetDebugVisualizerCamera(0.4, 60, 90, 
			p.getBasePositionAndOrientation(uid)[0])




