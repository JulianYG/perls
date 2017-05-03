import pybullet as p
import numpy as np
from bullet.agents.core.tool import *
from bullet.utils.enum import *

class Robot(Tool):

	def __init__(self, enableForceSensor):

		super(Robot, self).__init__(enableForceSensor)
		self.THRESHOLD = 1.3
		# Common type of grippers for all robots
		self.gripper_urdf = 'gripper/wsg50_one_motor_gripper_new_free_base.sdf'

		self.GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
		self.GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, 
			-0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]
		self.JOINT_DAMP = [.1] * self.nDOF

	def get_tool_ids(self):
		return self.arms

	def get_tool_pose(self, tool_id):
		# End effector pose for robot
		# Returns linkWorldPos, linkWorldOrn
		return self.get_tool_link_state((tool_id, -1))

	def control(self, event, ctrl_map):

		ctrl_id = event[0]
		arm_id = ctrl_map[ARM][ctrl_id]
		gripper_id = ctrl_map[GRIPPER][ctrl_id]
		
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
			p.setJointMotorControl2(gripper, i, POS_CTRL, 
				targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

	def release(self, gripper):
		for i in range(p.getNumJoints(gripper)):
			p.setJointMotorControl2(gripper, i, POS_CTRL, 
				targetPosition=self.GRIPPER_REST_POS[i], force=50)

	def slide_grasp(self, gripper, event):

		#TODO: Add slider for the grippers
		analog = event[3]
		if event[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, POS_CTRL, 
					targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

		if event[6][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, POS_CTRL, 
					targetPosition=self.GRIPPER_REST_POS[i], force=50)	

	def reach(self, arm_id, eef_pos, eef_orien, fixed, ctrl=POS_CTRL, 
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
				return

			# Only need links 1- 5, no need for joint 4-6 with pure position IK
			for i in range(len(joint_pos) - 3):
				p.setJointMotorControl2(arm_id, i, ctrl, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=pos_gain, 
					velocityGain=1.0, force=self.MAX_FORCE)
			
			x, y, _ = p.getEulerFromQuaternion(eef_orien)

			#TO-DO: add wait till fit

			# Link 4 needs protection
			if self.LOWER_LIMITS[self.nDOF - 1] < x < self.UPPER_LIMITS[self.nDOF - 1]:	
				p.setJointMotorControl2(arm_id, self.nDOF - 1, 
					ctrl, 
					targetPosition=x, targetVelocity=0, positionGain=pos_gain, 
					velocityGain=1.0, force=self.MAX_FORCE)
			else:
				p.setJointMotorControl2(arm_id, self.nDOF - 1, 
					ctrl, 
					targetPosition=joint_pos[self.nDOF - 1], targetVelocity=0, 
					positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)

				self.mark('Warning: you are flipping arm link {}'.format(self.nDOF - 1), 
					p.getLinkState(arm_id, 0)[0], time=1.5)
				raise IllegalOperation(self.nDOF - 1)

			if self.LOWER_LIMITS[self.nDOF - 2] < y < self.UPPER_LIMITS[self.nDOF - 2]:
				p.setJointMotorControl2(arm_id, self.nDOF - 2, ctrl, 
					targetPosition=-y, targetVelocity=0, positionGain=pos_gain, 
					velocityGain=1.0, force=self.MAX_FORCE)
			else:
				p.setJointMotorControl2(arm_id, self.nDOF - 2, ctrl, 
					targetPosition=joint_pos[self.nDOF - 2], targetVelocity=0, 
					positionGain=pos_gain, velocityGain=1.0, force=self.MAX_FORCE)

				self.mark('Warning: you are flipping arm link {}'.format(self.nDOF - 2), 
					p.getLinkState(arm_id, 1)[0], time=1.5)
				raise IllegalOperation(self.nDOF - 2)

	def _load_tools(self, positions):
		# Gripper ID to arm ID
		for pos in positions:
			self.arms.append(p.loadURDF(self.arm_urdf, 
				pos, [0, 0, 0, 1], useFixedBase=True))
			self.grippers.append(p.loadSDF(self.gripper_urdf)[0])

		# Setup initial conditions for both arms
		for arm in self.arms:
			self._reset_robot(arm)

		# Setup initial conditions for both grippers
		for gripper in self.grippers:
			self._reset_robot_gripper(gripper)
			
		# Setup constraints on grippers
		for arm, gripper in zip(self.arms, self.grippers):
			self.constraints.append(p.createConstraint(arm, self.nDOF - 1, 
				gripper, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0], 
				parentFrameOrientation=[0, 0, 0, 1]))
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
				p.setJointMotorControl2(robot, jointIndex, POS_CTRL, 
					targetPosition=self.REST_POSE[jointIndex], 
					targetVelocity=0, positionGain=0.03, velocityGain=1,
					force=self.MAX_FORCE)

	def _reset_robot(self, robot):
		for jointIndex in range(p.getNumJoints(robot)):
			# p.resetJointState(robot, jointIndex, self.REST_POSE[jointIndex])
			p.setJointMotorControl2(robot, jointIndex, POS_CTRL, 
				self.REST_POSE[jointIndex], 0)

	def _reset_robot_gripper(self, robot_gripper):
		for jointIndex in range(p.getNumJoints(robot_gripper)):
			# p.resetJointState(robot_gripper, jointIndex, self.GRIPPER_REST_POS[jointIndex])
			p.setJointMotorControl2(robot_gripper, jointIndex, 
				POS_CTRL, self.GRIPPER_REST_POS[jointIndex], 0)




