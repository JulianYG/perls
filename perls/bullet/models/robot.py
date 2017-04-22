import pybullet as p
import numpy as np
from bullet.models.core.tool import Tool
from bullet.util import *

class Robot(Tool):

	def __init__(self, pos, enableForceSensor):

		super(Robot, self).__init__(pos, enableForceSensor)
		self.THRESHOLD = 1.3
		self.pos = pos

	def get_tool_ids(self):
		return self.arms

	def get_tool_pose(self, tool_id, velocity=0):
		# End effector pose for robot
		if velocity:
			state = p.getLinkState(tool_id, p.getNumJoints(tool_id) - 1, 1)
			return np.array([list(state[0]), 
				list(state[1])] + [list(state[-2]), list(state[-1])])
		else:
			state = p.getLinkState(tool_id, p.getNumJoints(tool_id) - 1)
			return [list(state[0]), list(state[1])]

	def control(self, event, ctrl_map):

		ctrl_id = event[0]
		arm_id = ctrl_map[ARM][ctrl_id]
		gripper_id = ctrl_map[GRIPPER][ctrl_id]
		
		self.slide_grasp(gripper_id, event)

		#TODO: make this as another function (mark event)
		# Add user interaction for task completion
		if (event[self.BUTTONS][1] & p.VR_BUTTON_WAS_TRIGGERED):
			# p.resetSimulation()
			# p.removeAllUserDebugItems()
			p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
			# Can add line for mark here
			# so that in saved csv file, we know when one task is complete	

		# Allows robot arm control by VR controllers
		if self.get_tool_control_deviation(arm_id, event[1]) < self.THRESHOLD:
			self._engage(arm_id, event)
			return 0
		else:
			self._disengage(arm_id, event)
			return -1

	def grip(self, gripper):
		for i in range(p.getNumJoints(gripper)):
			p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
				targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

	def release(self, gripper):
		for i in range(p.getNumJoints(gripper)):
			p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
				targetPosition=self.GRIPPER_REST_POS[i], force=50)

	def slide_grasp(self, gripper, event):

		#TODO: Add slider for the grippers
		analog = event[3]
		if event[self.BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

		if event[self.BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.GRIPPER_REST_POS[i], force=50)	

	def _engage(self, robot, controller_event):
		controller_pos = controller_event[1]
		controller_orn = controller_event[self.ORIENTATION]
		targetPos = controller_pos
		eef_orn = controller_orn
		if controller_event[self.BUTTONS][32] & p.VR_BUTTON_IS_DOWN:
			if self.FIX:
				self.reach(robot, targetPos, (0, 1, 0, 0), self.FIX)
			else:
				self.reach(robot, targetPos, controller_orn, self.FIX)

	def _disengage(self, robot, controller_event):

		if controller_event[self.BUTTONS][32] & p.VR_BUTTON_IS_DOWN:
			for jointIndex in range(p.getNumJoints(robot)):
				p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, 
					targetPosition=self.REST_POSE[jointIndex], 
					targetVelocity=0, positionGain=0.03, velocityGain=1,
					force=self.MAX_FORCE)

	def _reset_robot(self, robot):
		for jointIndex in range(p.getNumJoints(robot)):
			p.resetJointState(robot, jointIndex, self.REST_POSE[jointIndex])
			p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, 
				self.REST_POSE[jointIndex], 0)

	def _reset_robot_gripper(self, robot_gripper):
		for jointIndex in range(p.getNumJoints(robot_gripper)):
			p.resetJointState(robot_gripper, jointIndex, self.GRIPPER_REST_POS[jointIndex])
			p.setJointMotorControl2(robot_gripper, jointIndex, 
				p.POSITION_CONTROL, self.GRIPPER_REST_POS[jointIndex], 0)




