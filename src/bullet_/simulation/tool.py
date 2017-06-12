
__package__ = 'bullet_.simulation'

import pybullet as p
import numpy as np

import sys

from .world import World

from .utils.misc import Constant
from .utils import handler, misc


class Tool(World):

	def __init__(self, enableForceSensor):

		super(Tool, self).__init__()
		self.THRESHOLD = 1.3
		self.FIX = False
		self.close_grip = False
		self.has_force_sensor = enableForceSensor

	def set_force_sensor(self):
		self.has_force_sensor = True

	def set_virtual_controller(self, controllers):
		self.controllers = controllers

	def get_tool_control_deviation(self, tool_id, pos):
		eef_id = p.getNumJoints(tool_id) - 1
		return misc.get_distance(p.getLinkState(tool_id, eef_id)[0], pos)

	def redundant_control(self):
		return len(self.controllers) > max(len(self.grippers), 
			len(self.arms), len(self.constraints))

	def create_control_mappings(self):
		control_map, obj_map = {}, {}
		if self.arms:
			control_map[Constant.ARM] = dict(zip(self.controllers, self.arms))
			obj_map[Constant.ARM] = dict(zip(self.arms, self.controllers))
		if self.grippers:
			control_map[Constant.GRIPPER] = dict(zip(self.controllers, self.grippers))
			obj_map[Constant.GRIPPER] = dict(zip(self.grippers, self.controllers))
		if self.constraints:
			control_map[Constant.CONSTRAINT] = dict(zip(self.controllers, 
				self.constraints))
		return control_map, obj_map

	def set_tool_joint_states(self, tool_ids, vals, ctrl=Constant.POS_CTRL, **kwargs):
		"""
		Given (tool_id, joint_values) tuple, set the joints of tool with 
		given values and specified control type. If the joint value is None,
		the corresponding joint will remain previous state
		TORQ_CTRL, VEL_CTRL
		"""
		pd, vd, f = 0.05, 1.0, self.MAX_FORCE
		if not isinstance(tool_ids, list):
			tool_ids = [tool_ids]

		vals = np.reshape(vals, (len(tool_ids), self.nDOF))
		assert len(tool_ids) == len(vals), \
			'Specified tool ids not matching given joint values'

		if 'positionGain' in kwargs:
			pd = kwargs['positionGain']
		if 'velocityGain' in kwargs:
			vd = kwargs['velocityGain']

		for tool_id, value in zip(tool_ids, vals):
			num_joints = p.getNumJoints(tool_id)

			assert num_joints == len(value), \
				'Given tool not having right number of joints as given values'

			for jointIndex in range(num_joints):
				# Allow None or NaN in array
				if not value[jointIndex]:
					continue

				if ctrl is Constant.VEL_CTRL:
					p.setJointMotorControl2(tool_id, jointIndex, ctrl, 
						targetVelocity=value[jointIndex], 
						positionGain=pd, velocityGain=vd, force=f)

				elif ctrl is Constant.TORQ_CTRL:
					p.setJointMotorControl2(tool_id, jointIndex, ctrl,
						force=value[jointIndex])
				
				elif ctrl is Constant.POS_CTRL:
					p.setJointMotorControl2(tool_id, jointIndex, ctrl,
						targetPosition=value[jointIndex], targetVelocity=0, 
						positionGain=pd, velocityGain=vd, force=f)
				else:
					raise NotImplementedError('Unsupported control type')

	def get_tool_joint_state(self, tool_joint_idx):
		"""
		Given (tool_id, joint) index tuple, return given joint 
		state of given tool.
		Return tuples of ((Velocity, NaN, Position), Forces)
		The NaN column is just used to match the convention of CTRL TYPE
		If sensor not activated, the second item will always be list of six 0.0's
		"""
		joint_state = p.getJointState(tool_joint_idx[0], tool_joint_idx[1])
		reaction_forces = [0.] * 6
		if self.has_force_sensor:
			reaction_forces = joint_state[2]
		return [joint_state[1], 0, joint_state[0]], reaction_forces

	def get_tool_joint_states(self, tool_idx):
		"""
		Given tool index, return a tuple of (joint states, reaction force) 
		of that tool
		Note it returns a tuple of joint states and reaction forces, 
		where the second term is 0's when force sensor is not enabled
		"""	
		if isinstance(tool_idx, list):
			joint_states = [[self.get_tool_joint_state((t, 
				i))[0] for i in range(p.getNumJoints(t))] for t in tool_idx]
			reaction_forces = [[self.get_tool_joint_state((t, 
				i))[1] for i in range(p.getNumJoints(t))] for t in tool_idx]
		else:
			joint_states = [self.get_tool_joint_state((tool_idx, 
				i))[0] for i in range(p.getNumJoints(tool_idx))]
			reaction_forces = [self.get_tool_joint_state((tool_idx, 
				i))[1] for i in range(p.getNumJoints(tool_idx))]
		return np.array(joint_states), np.array(reaction_forces)

	def get_tool_link_state(self, tool_link_idx, velocity=0):
		"""
		Returns pos, orn, link_velocity
		Given (tool, joint) index tuple, return given link 
		state of given tool. -1 indicates end effector
		Returns linkWorldPos, linkWorldOrn (x,y,z) radians, linkWorldLinearVel, linkWorldAngularVel
		# If velocity=0, the last two columns will be zeros
		"""
		if tool_link_idx[1] == -1:
			tool_link_idx = (tool_link_idx[0], 
				p.getNumJoints(tool_link_idx[0]) - 1)
		if velocity:
			link_state = p.getLinkState(tool_link_idx[0], tool_link_idx[1], 1)
		else:
			link_state = p.getLinkState(tool_link_idx[0], tool_link_idx[1], 0)

		linear_vel = [0.] * 3
		angular_vel = [0.] * 3
		if velocity:
			linear_vel = list(link_state[-2])
			angular_vel = list(link_state[-1])
		return np.array([list(link_state[0]), p.getEulerFromQuaternion(list(link_state[1])), 
				linear_vel, angular_vel])

	def get_tool_link_states(self, tool_idx, velocity=0):
		"""
		TODO: Use GUIGUIGUI DIRECTDIRECT
		Combining Keyboard & VR

		Returns pos, orn, link_velocity
		Given tool index, return all link states of given tool
		"""	
		if isinstance(tool_idx, list):
			link_states = [[self.get_tool_link_state((t, i), 
				velocity) for i in range(p.getNumJoints(t))] for t in tool_idx]
		else:
			link_states = [self.get_tool_link_state((tool_idx, i), 
				velocity) for i in range(p.getNumJoints(tool_idx))]
		return link_states

	def get_tool_poses(self, tool_ids):
		return np.array([self.get_tool_pose(t) for t in tool_ids])

	def control(self, event, ctrl_map):
		raise NotImplementedError("Each tool agent must re-implement this method.")

	def reach(self, tool_id, eef_pos, eef_orien, fixed, expedite=False):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def slide_grasp(self, gripper, controller_event):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def grip(self, gripper):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def release(self, gripper):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def _set_camera(self, uid):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def get_tool_ids(self):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def get_tool_pose(self, tool_id):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def _load_tools(self, pos, reset):
		raise NotImplementedError('Each tool agent must re-implement this method.')


class PR2(Tool):

	def __init__(self, pos, enableForceSensor=False):

		super(PR2, self).__init__(enableForceSensor)
		self.gripper_max_joint = 0.550569
		self.THRESHOLD = 1.0
		self.completed_task = {}
		self.boxes = {}
		self.positions = [[0.5, ypos, 0.7] for ypos in pos]
		self.nDOF = 4

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
		constraint_id = ctrl_map[Constant.CONSTRAINT][ctrl_id]
		gripper_id = ctrl_map[Constant.GRIPPER][ctrl_id]
		self._set_camera(gripper_id)

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
		if not self.close_grip:
			p.setJointMotorControl2(gripper, 0, Constant.POS_CTRL, 
				targetPosition=0, force=5.0)
			p.setJointMotorControl2(gripper, 2, Constant.POS_CTRL, 
				targetPosition=0, force=5.0)
			self.close_grip = True

	def release(self, gripper):
		"""
		A forced release without analog slide.
		"""
		if self.close_grip:
			p.setJointMotorControl2(gripper, 0, Constant.POS_CTRL, 
				targetPosition=self.gripper_max_joint, force=10)
			p.setJointMotorControl2(gripper, 2, Constant.POS_CTRL, 
				targetPosition=self.gripper_max_joint, force=10)
			self.close_grip = False

	def slide_grasp(self, gripper, event):
		# Setup gliders
		analog_slide = self.gripper_max_joint * (1 - event[3])
		p.setJointMotorControl2(gripper, 0, Constant.POS_CTRL, 
			targetPosition=analog_slide, force=5.0)
		p.setJointMotorControl2(gripper, 2, Constant.POS_CTRL, 
			targetPosition=analog_slide, force=5.0)

	def _load_tools(self, positions, reset):

		for i in range(len(positions)):
			pos = positions[i]
			pr2_gripper = p.loadURDF("pr2_gripper.urdf", pos, [0, 0, 0, 1])
			# Setup the pr2_gripper
			jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
			if reset:
				for jointIndex in range(p.getNumJoints(pr2_gripper)):
					p.resetJointState(pr2_gripper, jointIndex,jointPositions[jointIndex])

			# Use -1 for the base, constrained within controller
			pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED,
				[0, 0, 0], [0, 0, 0], pos)

			self.grippers.append(pr2_gripper)
			self.constraints.append(pr2_cid)
			self.name_dic[pr2_gripper] = 'pr2_{}'.format(i)
		self.solo = len(self.grippers) == 1

	def _set_camera(self, uid):
		p.resetDebugVisualizerCamera(0.4, 75, -40, 
			p.getBasePositionAndOrientation(uid)[0])






