import pybullet as p
from bullet.models.core.tool import Tool
import numpy as np
from bullet.util import *

class PR2(Tool):

	def __init__(self, pos, enableForceSensor=False):

		super(PR2, self).__init__(pos, enableForceSensor)
		self.gripper_max_joint = 0.550569
		self.THRESHOLD = 1.0
		self.completed_task = {}
		self.boxes = {}

	def setup_scene(self, task):
		"""
		Basic scene needed for running tasks
		"""
		self.load_default_env()
		self._load_tools(self.pos)
		self.default_obj_cnt = p.getNumBodies()
		self._load_task(task)
		self.loaded_obj = range(self.default_obj_cnt, p.getNumBodies())
		p.setGravity(0, 0, -9.81)

		#TODO: think about extracting this bounding box out to avoid repeating code if 
		# this gripper works, abandon demoVR
		# Use loadArm=True/False for demoVR load_default_env 
		# for obj in self.task:
		# 	iD = p.loadURDF(*obj)
		# 	p.addUserDebugText(str(iD - self.obj_cnt), 
		# 		p.getBasePositionAndOrientation(iD)[0], textSize=8, lifeTime=0)
		# #TODO: add labels
		# self._load_boxes(numOfBoxes=9)

	def get_tool_ids(self):
		return self.grippers

	def get_tool_pose(self, tool_id, velocity=0):
		state = p.getBasePositionAndOrientation(tool_id)
		if velocity:
			lin_vel, ang_vel = p.getBaseVelocity(tool_id)
			return np.array([list(state[0]), 
				list(state[1])] + [lin_vel, ang_vel])
		else:
			state = p.getLinkState(tool_id, p.getNumJoints(tool_id) - 1)
			return np.array([list(state[0]), list(state[1])])

	def control(self, event, ctrl_map):
		"""
		Handles one gripper at a time
		"""
		ctrl_id = event[0]
		constraint_id = ctrl_map[CONSTRAINT][ctrl_id]
		gripper_id = ctrl_map[GRIPPER][ctrl_id]

		self.reach(constraint_id, event[1], event[self.ORIENTATION], fixed=False)
		self.slide_grasp(gripper_id, event)

		# addMark()
		if (event[self.BUTTONS][1] & p.VR_BUTTON_WAS_TRIGGERED):
			p.addUserDebugText('One Item Inserted', (1.7, 0, 1), (255, 0, 0), 12, 10)
		
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
		p.setJointMotorControl2(gripper, 0, p.POSITION_CONTROL, 
			targetPosition=0, force=5.0)
		p.setJointMotorControl2(gripper, 2, p.POSITION_CONTROL, 
			targetPosition=0, force=5.0)

	def release(self, gripper):
		"""
		A forced release without analog slide.
		"""
		p.setJointMotorControl2(gripper, 0, p.POSITION_CONTROL, 
			targetPosition=self.gripper_max_joint, force=10)
		p.setJointMotorControl2(gripper, 2, p.POSITION_CONTROL, 
			targetPosition=self.gripper_max_joint, force=10)

	def slide_grasp(self, gripper, event):
		# Setup gliders
		analog_slide = self.gripper_max_joint * (1 - event[3])
		p.setJointMotorControl2(gripper, 0, p.POSITION_CONTROL, 
			targetPosition=analog_slide, force=5.0)
		p.setJointMotorControl2(gripper, 2, p.POSITION_CONTROL, 
			targetPosition=analog_slide, force=5.0)

	def _load_tools(self, pos):

		for ypos in pos:
			pr2_gripper = p.loadURDF("pr2_gripper.urdf", [0.5, ypos, 0.7], [0, 0, 0, 1])
			# Setup the pr2_gripper
			jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
			for jointIndex in range(p.getNumJoints(pr2_gripper)):
				p.resetJointState(pr2_gripper, jointIndex,jointPositions[jointIndex])

			# Use -1 for the base, constrained within controller
			pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED,
				[0, 0, 0], [0, 0, 0], [0.500000, ypos, 0.700000])

			self.grippers.append(pr2_gripper)
			self.constraints.append(pr2_cid)

	def _check_task(self):
		# Only check boundaries for objects in task
		for obj in range(self.obj_cnt, p.getNumBodies()):
			if obj not in self.completed_task:

				obj_pos = p.getBasePositionAndOrientation(obj)[0]
				bound = self.boxes[obj - self.obj_cnt]

				if self._fit_boundary(obj_pos, obj, bound):
					p.addUserDebugText('Finished', obj_pos, [1, 0, 0], lifeTime=5.)
					self._fit_routine(obj_pos, obj, bound)
					
	def _fit_routine(self, obj_pos, obj, boundary):
		self.complete_task[obj] = True
		# Change color
		for line, vertex in self.boxes[boundary]:
			p.removeUserDebugItem(line)
			p.addUserDebugLine(vertex[0], vertex[1], lineColorRGB=(0, 1, 0), lifeTime=0)
		# Hardcoded fact
		tableID = 14
		tablePosition = p.getBasePositionAndOrientation(tableID)[0]
		relPosition = [obj_pos[i] - tablePosition[i] for i in range(3)]
		# Add constraint
		p.createConstraint(tableID, 0, obj, 0, p.JOINT_POINT2POINT, [0, 0, 0], 
			relPosition, [0, 0, 0])

	def _fit_boundary(self, position, obj, boundary):

		table_top = [i[2] for i in p.getContactPoints(14)]	# hardcoded table
		return boundary[0][0] < position[0] < boundary[1][0] and boundary[0][1] < position[1] < boundary[1][1]\
			and obj in table_top
			
	def _load_boxes(self, startPos=(1.0, -0.7), numOfBoxes=3, size=0.07, interval=0.01, 
		height=0.63, color=(1, 0, 0)):
		"""
		Currently display the box shapes on the table surface
		"""
		for i in range(numOfBoxes):
			a_i_x = startPos[0] + i * (size + interval)
			a_i_y = startPos[1] 
			b_i_x = startPos[0] + i * (size + interval) + size
			b_i_y = startPos[1] + size
			self.boxes[i] = (((a_i_x, a_i_y), (b_i_x, b_i_y)))

		def construct_box(box_num):
			diag_a, diag_b = self.boxes[box_num]
			ax, ay = diag_a
			bx, by = diag_b
			v_a = (ax, ay, height)
			v_b = (bx, ay, height)
			v_c = (bx, by, height)
			v_d = (ax, by, height)
			a = p.addUserDebugLine(v_a, v_b, lineColorRGB=color, lifeTime=0)
			b = p.addUserDebugLine(v_b, v_c, lineColorRGB=color, lifeTime=0)
			c = p.addUserDebugLine(v_c, v_d, lineColorRGB=color, lifeTime=0)
			d = p.addUserDebugLine(v_d, v_a, lineColorRGB=color, lifeTime=0)
			# Label the box
			p.addUserDebugText(str(box_num), ((ax + bx) / 2., (ay + by) / 2., height), 
				textSize=8, lifeTime=0)
			# Keep track of the box region
			self.boxes[box_num] = (
								   (a, (v_a, v_b)), 
								   (b, (v_b, v_c)),
								   (c, (v_c, v_d)),
								   (d, (v_d, v_a))
								  )
		for k in self.boxes.keys():
			construct_box(k)




