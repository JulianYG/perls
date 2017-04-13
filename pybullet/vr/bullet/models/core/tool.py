import pybullet as p
from bullet.models.core.scene import Scene

class Tool(Scene):

	def __init__(self, pos, enableForceSensor):

		super(Tool, self).__init__(enableForceSensor, pos)
		self.THRESHOLD = 1.3
		self.pos = pos

	def setup_scene(self, task):
		raise NotImplementedError('Each tool class should implement this method.')

	def control(self, event, ctrl_map):
		raise NotImplementedError("Each tool model must re-implement this method.")

	def get_tool_joint_states(self):
		raise NotImplementedError('Each tool class should implement this method.')

	def get_tool_link_states(self, link_idx):
		"""
		Returns pos, orn, link_velocity
		"""
		raise NotImplementedError('Each tool class should implement this method.')

	def control(self, event, ctrl_map):
		raise NotImplementedError('Each tool class should implement this method.')

	def reach(self, tool_id, eef_pos, eef_orien, fixed):
		raise NotImplementedError('Each tool class should implement this method.')

	def grasp(self, gripper, controller_event):
		raise NotImplementedError('Each tool class should implement this method.')

	def _load_tools(self, pos):
		raise NotImplementedError('Each tool class should implement this method.')

	def add_marker(self, arm_id, color):
		#TODO
		pass

	def get_arm_ids(self):
		return self.arms

	def set_force_sensor(self):
		self.has_force_sensor = True

	def set_virtual_controller(self, controllers):
		self.controllers = controllers

	def get_tool_control_deviation(self, tool_id, pos):
		eef_id = p.getNumJoints(tool_id) - 1
		return self._get_distance(p.getLinkState(tool_id, eef_id)[0], pos)

	def create_control_mappings(self):
		control_map = {}
		if self.grippers:
			control_map['gripper'] = dict(zip(self.controllers, self.grippers))
		if self.arms:
			control_map['arm'] = dict(zip(self.controllers, self.arms))
		if self.constraints:
			control_map['constraint'] = dict(zip(self.controllers, self.constraints))
		return control_map

	def redundant_control(self):
		return len(self.controllers) > max(len(self.grippers), 
			len(self.arms), len(self.constraints))

	def set_tool_states(self, tool_ids, vals, ctrl_type='pos'):

		if ctrl_type == 'pos':
			for tool_id, val in zip(tool_ids, vals):
				for jointIndex in range(p.getNumJoints(tool_id)):
					p.setJointMotorControl2(tool_id, jointIndex, p.POSITION_CONTROL,
						targetPosition=pos[jointIndex], targetVelocity=0, positionGain=0.05, 
						velocityGain=1.0, force=self.MAX_FORCE)
		elif ctrl_type == 'vel':
			for tool_id, val in zip(tool_ids, vals):
				for jointIndex in range(p.getNumJoints(tool_id)):
					p.setJointMotorControl2(tool_id, jointIndex, p.VELOCITY_CONTROL,
						targetVelocity=val[jointIndex], force=self.MAX_FORCE)
		else:
			raise NotImplementedError('Cannot recognize current control type: ' + ctrl_type)

	def _get_distance(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist


	