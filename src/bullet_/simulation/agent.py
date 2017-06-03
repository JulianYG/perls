import pybullet as p
import numpy as np
import sys, os
from os.path import join as pjoin
sys.path.append(os.path.abspath(pjoin(os.path.dirname(__file__))))


from utils.enum import *
from utils.classes import *
from utils.helpers import get_distance
from world import World

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
		TORQ_CTRL, VEL_CTRL
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

	def _set_camera(self, uid):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def get_tool_ids(self):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def get_tool_pose(self, tool_id):
		raise NotImplementedError('Each tool agent must re-implement this method.')

	def _load_tools(self, pos):
		raise NotImplementedError('Each tool agent must re-implement this method.')



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

		for i in range(len(positions)):
			pos = positions[i]
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
			self.name_dic[pr2_gripper] = 'pr2_{}'.format(i)
		self.solo = len(self.grippers) == 1

	def _set_camera(self, uid):
		p.resetDebugVisualizerCamera(0.4, 45, 90, 
			p.getBasePositionAndOrientation(uid)[0])

class Arm(Tool):

	def __init__(self, enableForceSensor,
		gripper_file='gripper/wsg50_one_motor_gripper_new_free_base.sdf'):

		super(Arm, self).__init__(enableForceSensor)
		self.THRESHOLD = 1.3
		# Common type of grippers for all robots
		self.gripper_file = gripper_file

		self.GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
		self.GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, 
			-0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]
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
		arm_id = ctrl_map[ARM][ctrl_id]
		gripper_id = ctrl_map[GRIPPER][ctrl_id]
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
				return joint_pos

			# Only need links 1- 5, no need for joint 4-6 with pure position IK
			for i in range(len(joint_pos) - 3):
				p.setJointMotorControl2(arm_id, i, ctrl, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=pos_gain, 
					velocityGain=1.0, force=self.MAX_FORCE)
			
			x, y, _ = p.getEulerFromQuaternion(eef_orien)

			#TO-DO: add wait till fit

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
				raise IllegalOperation(self.nDOF - 1)

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
				raise IllegalOperation(self.nDOF - 2)
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
				gripper, 0, p.JOINT_FIXED, [0,0,0], [0,0, self.ee_offset], [0,0,0], 
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






