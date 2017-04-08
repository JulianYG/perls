import pybullet as p
from bullet.models.core.physics import Scene


class Robot(Scene):

	def __init__(self, pos, enableForceSensor):

		super(Robot, self).__init__(enableForceSensor, pos)
		self.THRESHOLD = 1.3
		self.pos = pos

	def setup_scene(self, task):
		"""
		Basic scene needed for running tasks
		"""
		self.load_basic_env()
		for obj in task:
			p.loadURDF(*obj)
		self.obj_cnt = p.getNumBodies()
		p.setGravity(0, 0, -9.81)

	def get_robot_states(self):
		states = []
		for arm in self.arms:
			joints = range(p.getNumJoints(arm))
			arm_state = [p.getJointState(i)[:2] for i in joints]
			if self.has_force_sensor:
				arm_state += [p.getJointState(i)[2] for i in joints]
			states.append(arm_state)
		# Return data points and label
		return states, self.arms

	def get_tool_info(self, link_idx):
		"""
		Returns pos, orn, link_velocity
		"""
		link_pose = []
		for arm in self.arms:
			if isinstance(link_idx, list):
				link_pose.append([(p.getLinkState(arm, i)[0],
				   p.getLinkState(arm, i)[1], 
				   p.getLinkState(arm, i, 1)[-2]) for i in link_idx])
			elif link_idx == -1:
				eef_idx = p.getNumJoints(arm) - 1
				link_pose.append((p.getLinkState(arm, eef_idx)[0],
					p.getLinkState(arm, eef_idx)[1], 
					p.getLinkState(arm, eef_idx, 1)[-2]))
			else:
				link_pose.append((p.getLinkState(arm, link_idx)[0],
					p.getLinkState(arm, link_idx)[1],
					p.getLinkState(arm, link_idx, 1)[-2]))

		return link_pose

	def get_tool_control_deviation(self, arm_id, pos):
		eef_id = p.getNumJoints(arm_id) - 1
		return self._get_distance(p.getLinkState(arm_id, eef_id)[0], pos)

	def control(self, event, ctrl_map):

		ctrl_id = event[0]
		arm_id = ctrl_map['arm'][ctrl_id]
		gripper_id = ctrl_map['gripper'][ctrl_id]
		
		self.grasp(gripper_id, event)

		#TODO: make this as another function (mark event)
		# Add user interaction for task completion
		if (event[self.BUTTONS][1] & p.VR_BUTTON_WAS_TRIGGERED):
			# p.resetSimulation()
			# p.removeAllUserDebugItems()
			p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
			# Can add line for mark here
			# so that in saved csv file, we know when one task is complete	

		sq_len = self.get_tool_control_deviation(arm_id, event[1])

		# Allows robot arm control by VR controllers
		if sq_len < self.THRESHOLD * self.THRESHOLD:
			self._engage(arm_id, event)
			return 0
		else:
			self._disengage(arm_id, event)
			return -1

	def set_robot_pos(self, arm_ids, vals, ctrl_type='pos'):

		if ctrl_type == 'pos':
			for arm_id, val in zip(arm_ids, vals):
				for jointIndex in range(p.getNumJoints(arm_id)):
					p.setJointMotorControl2(arm_id, jointIndex, p.POSITION_CONTROL,
						targetPosition=pos[jointIndex], targetVelocity=0, positionGain=0.05, 
						velocityGain=1.0, force=self.MAX_FORCE)
		elif ctrl_type == 'vel':
			for arm_id, val in zip(arm_ids, vals):
				for jointIndex in range(p.getNumJoints(arm_id)):
					p.setJointMotorControl2(arm_id, jointIndex, p.VELOCITY_CONTROL,
						targetVelocity=val[jointIndex], force=self.MAX_FORCE)
		else:
			raise NotImplementedError('Cannot recognize current control type: ' + ctrl_type)

	def grasp(self, gripper, controller_event):

		#TODO: Add slider for the grippers
		if controller_event[self.BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.GRIPPER_CLOZ_POS[i], force=50)

		if controller_event[self.BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.GRIPPER_REST_POS[i], force=50)	

	def reach(self, arm_id, eef_pos, eef_orien, fixed):
		raise NotImplementedError('Each robot class should implement this method.')

	def add_marker(self, arm_id, color):
		#TODO
		pass

	def _get_distance(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist

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
					targetPosition=self.REST_JOINT_POS[jointIndex], 
					targetVelocity=0, positionGain=0.03, velocityGain=1,
					force=self.MAX_FORCE)

	def _reset_robot(self, robot):
		for jointIndex in range(p.getNumJoints(robot)):
			p.resetJointState(robot, jointIndex, self.REST_JOINT_POS[jointIndex])
			p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, 
				self.REST_JOINT_POS[jointIndex], 0)

	def _reset_robot_gripper(self, robot_gripper):
		for jointIndex in range(p.getNumJoints(robot_gripper)):
			p.resetJointState(robot_gripper, jointIndex, self.GRIPPER_REST_POS[jointIndex])
			p.setJointMotorControl2(robot_gripper, jointIndex, 
				p.POSITION_CONTROL, self.GRIPPER_REST_POS[jointIndex], 0)

