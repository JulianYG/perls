import pybullet as p
from bullet.models.core.physics import Scene


class Robot(Scene):

	def __init__(self, enableForceSensor):

		super(Robot, self).__init__(enableForceSensor)
		self.THRESHOLD = 1.3
		self.MAX_FORCE = 500
		self.pos = []

	def create_scene(self):
		"""
		Basic scene needed for running tasks
		"""
		p.resetSimulation()
		self.load_basic_env()
		self._load_robot(self.pos)

	def _load_robot(self, ypos):
		raise NotImplementedError("Each VR Robot Setup must re-implement this method.")

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

	def get_link_pos(self, link_idx):

		link_pos = []
		for arm in self.arms:
			if isinstance(link_idx, list):
				link_pos.append([p.getLinkState(arm, i, 1)[0] + \
					p.getLinkState(arm, i, 1)[-2] for i in link_idx])
			else:
				link_pos.append(p.getLinkState(arm, link_idx, 1)[0] + \
					p.getLinkState(arm, link_idx, 1)[-2])

		return link_pos

	def control(self, event, ctrl_map):

		ctrl_id = event[0]
		arm_id = ctrl_map['arm'][ctrl_id]
		gripper_id = ctrl_map['gripper'][ctrl_id]
		eef_id = p.getNumJoints(arm_id) - 1

		self.grasp(gripper_id, event)

		sq_len = self._get_distance(p.getLinkState(arm_id, eef_id)[0], event[1])

		# Allows robot arm control by VR controllers
		if sq_len < self.THRESHOLD * self.THRESHOLD:
			self._engage(arm_id, event)
		else:
			self._disengage(arm_id, event)

		# Add user interaction for task completion
		if (event[self.BUTTONS][1] & p.VR_BUTTON_WAS_TRIGGERED):
			# p.resetSimulation()
			# p.removeAllUserDebugItems()
			p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
			# Can add line for mark here
			# so that in saved csv file, we know when one task is complete		

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

	def _get_distance(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist

	def grasp(self, gripper, controller_event):

		#TODO: Add slider for the grippers
		if controller_event[self.BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.KUKA_GRIPPER_CLOZ_POS[i], force=50)

		if controller_event[self.BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(gripper)):
				p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, 
					targetPosition=self.KUKA_GRIPPER_REST_POS[i], force=50)	

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
				# p.resetJointState(robot, jointIndex, self.REST_JOINT_POS[jointIndex])
				p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, 
					self.REST_JOINT_POS[jointIndex], 0)

	def _reset_robot(self, robot):
		for jointIndex in range(p.getNumJoints(robot)):
			p.resetJointState(robot, jointIndex, self.REST_JOINT_POS[jointIndex])
			p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, 
				self.REST_JOINT_POS[jointIndex], 0)

	def _reset_robot_gripper(self, robot_gripper):
		for jointIndex in range(p.getNumJoints(robot_gripper)):
			p.resetJointState(robot_gripper, jointIndex, self.KUKA_GRIPPER_REST_POS[jointIndex])
			p.setJointMotorControl2(robot_gripper, jointIndex, 
				p.POSITION_CONTROL, self.KUKA_GRIPPER_REST_POS[jointIndex], 0)


