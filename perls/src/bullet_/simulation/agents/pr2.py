import pybullet as p
import numpy as np
from simulation.utils.enum import *
from simulation.agents.core.tool import Tool

class PR2(Tool):

	def __init__(self, pos, enableForceSensor=False):

		super(PR2, self).__init__(enableForceSensor)
		self.gripper_max_joint = 0.550569
		self.THRESHOLD = 1.0
		self.completed_task = {}
		self.boxes = {}
		self.positions = [[0.5, ypos, 0.7] for ypos in pos]

	def get_tool_ids(self):
		return self.grippers

	def get_tool_pose(self, tool_id):
		state = p.getBasePositionAndOrientation(tool_id)	
		return np.array([list(state[0]), list(p.getEulerFromQuaternion(state[1]))])

	def control(self, event, ctrl_map):
		"""
		Handles one gripper at a time
		"""
		ctrl_id = event[0]
		constraint_id = ctrl_map[CONSTRAINT][ctrl_id]
		gripper_id = ctrl_map[GRIPPER][ctrl_id]
		self.reach(constraint_id, event[1], event[2], fixed=False)
		self.slide_grasp(gripper_id, event)
		
		return 0 if self.get_tool_control_deviation(gripper_id, 
			event[1]) <= self.THRESHOLD * self.THRESHOLD else -1

	def reach(self, tool_id, eef_pos, eef_orien, fixed, expedite=False):
		# PR2 gripper follows VR controller, or keyboard		
		p.changeConstraint(tool_id, eef_pos, eef_orien,
		 	maxForce=self.MAX_FORCE)

	def grip(self, gripper):
		"""
		A hard grip without analog slide.
		"""
		p.setJointMotorControl2(gripper, 0, POS_CTRL, 
			targetPosition=0, force=5.0)
		p.setJointMotorControl2(gripper, 2, POS_CTRL, 
			targetPosition=0, force=5.0)

	def release(self, gripper):
		"""
		A forced release without analog slide.
		"""
		p.setJointMotorControl2(gripper, 0, POS_CTRL, 
			targetPosition=self.gripper_max_joint, force=10)
		p.setJointMotorControl2(gripper, 2, POS_CTRL, 
			targetPosition=self.gripper_max_joint, force=10)

	def slide_grasp(self, gripper, event):
		# Setup gliders
		analog_slide = self.gripper_max_joint * (1 - event[3])
		p.setJointMotorControl2(gripper, 0, POS_CTRL, 
			targetPosition=analog_slide, force=5.0)
		p.setJointMotorControl2(gripper, 2, POS_CTRL, 
			targetPosition=analog_slide, force=5.0)

	def _load_tools(self, positions):

		for pos in positions:
			pr2_gripper = p.loadURDF("pr2_gripper.urdf", pos, [0, 0, 0, 1])
			# Setup the pr2_gripper
			jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
			for jointIndex in range(p.getNumJoints(pr2_gripper)):
				p.resetJointState(pr2_gripper, jointIndex,jointPositions[jointIndex])

			# Use -1 for the base, constrained within controller
			pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED,
				[0, 0, 0], [0, 0, 0], pos)

			self.grippers.append(pr2_gripper)
			self.constraints.append(pr2_cid)
		self.solo = len(self.grippers) == 1

