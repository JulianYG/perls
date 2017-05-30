import pybullet as p
import numpy as np

from simulation.utils.enum import *
from simulation.utils.helpers import *

class World(object):
	"""
	The basic scene setup in VR 
	"""
	def __init__(self):
		"""
		Other subclasses may re-implement the constructor
		"""		
		self.controllers = None
		self.default_obj_cnt = 0

		self.hand = False
		self.grippers = []
		self.constraints = []
		self.arms = []
		self.loaded_obj = []
		self.VR_HAND_ID = None

		self.solo = None
		self.MAX_FORCE = 500

	def init_control(self):
		"""
		Load task for both recording and replay
		"""
		self.controllers = [e[0] for e in p.getVREvents()]
		return 1

	def setup_scene(self, scene, task, gui=True):
		self.init_control()
		self._load_env(scene)
		self._load_tools(self.positions)
		self.default_obj_cnt = p.getNumBodies()
		self._load_task(task)
		self.loaded_obj = range(self.default_obj_cnt, p.getNumBodies())
		p.setGravity(0, 0, -9.81)
		if not gui:
			for _ in range(100):
				p.stepSimulation()

	def mark(self, text, position, color=[255,0,0], font_size=8, time=10):
		try:
			p.addUserDebugText(text, position, color, font_size, time)
		except Exception:
			print(text)

	def get_loaded_obj(self):
		return self.loaded_obj

	def _load_env(self, env):
		for obj_pose in env:
			if len(obj_pose) == 3:
				ob = p.loadSDF(obj_pose[0])[0]
				p.resetBasePositionAndOrientation(ob, 
					obj_pose[1], obj_pose[2])
			else:
				p.loadURDF(
					obj_pose[0], obj_pose[1], 
					obj_pose[2], useFixedBase=obj_pose[3]
				)

	def _load_task(self, task):
		for obj_pose in task:
			if len(obj_pose) == 1:
				p.loadSDF(obj_pose[0])
			else:
				p.loadURDF(*obj_pose)


class Tool(World):

	def __init__(self, enableForceSensor):

		super(Tool, self).__init__()
		self.THRESHOLD = 1.3
		self.FIX = True
		self.has_force_sensor = enableForceSensor

	def set_force_sensor(self):
		self.has_force_sensor = True

	def set_virtual_controller(self, controllers):
		self.controllers = controllers

	def get_tool_control_deviation(self, tool_id, pos):
		eef_id = p.getNumJoints(tool_id) - 1
		return get_distance(p.getLinkState(tool_id, eef_id)[0], pos)

	def redundant_control(self):
		return len(self.controllers) > max(len(self.grippers), 
			len(self.arms), len(self.constraints))

	def create_control_mappings(self):
		control_map, obj_map = {}, {}
		if self.arms:
			control_map[ARM] = dict(zip(self.controllers, self.arms))
			obj_map[ARM] = dict(zip(self.arms, self.controllers))
		if self.grippers:
			control_map[GRIPPER] = dict(zip(self.controllers, self.grippers))
			obj_map[GRIPPER] = dict(zip(self.grippers, self.controllers))
		if self.constraints:
			control_map[CONSTRAINT] = dict(zip(self.controllers, 
				self.constraints))
		return control_map, obj_map

	def set_tool_joint_states(self, tool_ids, vals, ctrl=POS_CTRL, **kwargs):
		"""
		Given (tool_id, joint_values) tuple, set the joints of tool with 
		given values and specified control type. If the joint value is None,
		the corresponding joint will remain previous state
		"""
		pd, vd, f = 0.05, 1.0, self.MAX_FORCE
		if not isinstance(tool_ids, list):
			tool_ids = [tool_ids]
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
				if not value[jointIndex]:
					continue
				if ctrl is VEL_CTRL:
					p.setJointMotorControl2(tool_id, jointIndex, ctrl, 
						targetVelocity=value[jointIndex], 
						positionGain=pd, velocityGain=vd, force=f)
					continue
				if ctrl is TORQ_CTRL:
					p.setJointMotorControl2(tool_id, jointIndex, ctrl,
						positionGain=pd, velocityGain=vd, 
						force=value[jointIndex])
					continue
				p.setJointMotorControl2(tool_id, jointIndex, ctrl,
					targetPosition=value[jointIndex], targetVelocity=0, 
					positionGain=pd, velocityGain=vd, force=f)

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

	def get_tool_ids(self):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def get_tool_pose(self, tool_id):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def _load_tools(self, pos):
		raise NotImplementedError('Each tool agent must re-implement this method.')


	