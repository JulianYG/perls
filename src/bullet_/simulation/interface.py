__package__ = 'bullet_.simulation'

import pybullet as p
import time, sys

from numpy import array
import numpy as np

from .utils import handler
from .utils.misc import Constant, Key

from .checker import TaskChecker

class CtrlInterface(object):
	"""
	In charge of direct control with pybullet node. Can be either 
	direct VR / keyboard, etc. control on pybullet, or be a hub 
	that connects inputs from elsewhere such as ROS / openAI to 
	the pybullet node.
	"""
	def __init__(self, host, remote, task_name=''):
		self.remote = remote
		self.socket = host
		self.BTTN = 6
		self.ORTN = 2
		self.POSN = 1
		self.AAX = 3
		self.msg_holder = {}
		self.control_id = None
		self.task_name = task_name

	def start_remote_ctrl(self):
		self.remote = True

	def stop_remote_ctrl(self):
		self.remote = False

	def client_communicate(self, agent):
		raise NotImplementedError('Each interface must re-implement this method.')

	def server_communicate(self, agent, scene, task, gui=True):
		raise NotImplementedError('Each interface must re-implement this method.')

	def local_communicate(self, agent, gui=True):
		raise NotImplementedError('Each interface must re-implement this method.')

	def communicate(self, agent, scene, task, gui):
		self._checker = TaskChecker(self.task_name, agent.name_dic)
		if self.remote:
			self.server_communicate(agent, scene, task, gui=gui)
		else:
			self.local_communicate(agent, gui=gui)

	def close(self):
		if self.remote:
			self.socket.disconnect()

	def _event_loop(self, event, scene, task, agent, gui, skip=False):
		if skip:
			if event[0] == agent.controllers[1]:
				return 0
		# Hook handlers
		if event is Constant.RESET_HOOK:
			print('VR Client connected. Initializing reset...')
			p.setInternalSimFlags(0)
			p.resetSimulation()
			agent.constraints = []
			agent.setup_scene(scene, task, gui)
			agent.solo = len(agent.arms) == 1 or len(agent.grippers) == 1
			return 0 
		elif event is Constant.SHUTDOWN_HOOK:
			print('VR Client quit')
			return -1
		elif isinstance(event, tuple) and len(event) == 2:
			if event[0] is Constant.CTRL_HOOK and event[1]:
				agent.set_virtual_controller(event[1])
				# print('Received controller IDs: {}'.format(agent.controllers))
			return 0
		else:
			# Add user interaction for task completion
			# Can add line for mark here so that in saved csv file, 
			# we know when one task is complete	
			if (event[self.BTTN][1] & p.VR_BUTTON_WAS_TRIGGERED):
				task_monitor_handler(self.socket)
			return 1

	def _signal_loop(self, signal, agent, control_map, obj_map):

		# s = eval(signal)
		if signal is Constant.SHUTDOWN_HOOK:
			raise KeyboardInterrupt('Server invokes shutdown')
		elif signal is Constant.START_HOOK:
			print('Server is online')
		elif isinstance(signal, tuple) and signal[0] is Constant.WARNING_HOOK:
			agent.mark(*signal[1])
		else:	
			self._render_from_signal(agent, control_map, obj_map, signal)

	def _render_from_signal(self, agent, control_map, obj_map, signal):

		# print(sys.getsizeof(signal), 'signal package size')
		for obj, pose in signal.items():
			if obj == -1:
				self.control_id = pose
			elif (obj not in agent.grippers) and (obj not in agent.arms):
				p.resetBasePositionAndOrientation(obj, pose[0], pose[1])
			else:
				# Check if this is pr2 instance by checking arms (pr2 does not contain arms)
				if obj in agent.grippers:
					# Change the gripper constraint if obj is pr2 gripper (move it)
					if not agent.arms:
						p.changeConstraint(
							control_map[Constant.CONSTRAINT][obj_map[Constant.GRIPPER][obj]], 
							pose[0], pose[1], maxForce=agent.MAX_FORCE)

					# If robot arm instance, just set gripper close/release
					# The same thing for pr2 gripper
					# print(obj,'haha', p.getBodyInfo(obj), pose)
					agent.set_tool_joint_states([obj], [pose[2]], Constant.POS_CTRL)

				if obj in agent.arms:
					# print(obj, 'nono', p.getBodyInfo(obj), pose)
					agent.set_tool_joint_states([obj], [pose], Constant.POS_CTRL)

	def _msg_wrapper(self, tool_id, ids, agent, obj_map, ctrl=Constant.POS_CTRL):

		msg = {}
		if tool_id:
			msg[-1] = int(tool_id)
		for ID in ids:
			# If containing arms, send back arm and gripper info
			# since each arm must have one gripper
			if agent.grippers and ID in obj_map[Constant.GRIPPER]:
				msg[int(ID)] = list(p.getBasePositionAndOrientation(ID)[:2]) + \
								[agent.get_tool_joint_states(ID)[0][:, ctrl]]

			elif agent.arms and ID in obj_map[Constant.ARM]:
				msg[int(ID)] = agent.get_tool_joint_states(ID)[0][:, ctrl]
			else:
				msg[int(ID)] = list(p.getBasePositionAndOrientation(ID)[:2])

		# print(sys.getsizeof(msg), 'message package size')
		return msg


class ICmd(CtrlInterface):

	def __init__(self, host, remote, task_name):
		super(ICmd, self).__init__(host, remote, task_name)

	def server_communicate(self, agent, scene, task, gui=True):
		
		self.socket.connect_with_client()

		while True:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				try:
					if isinstance(event, dict):
						if 'pose' in event:
							agent.reach(event['id'], 
								event[pose][0], 
								event[pose][1], 
								fixed=False, 
								expedite=True)

				except handler.IllegalOperation as e:
					handler.illegal_operation_handler(e, self.socket)
					continue
			if not gui:
				p.stepSimulation()

	def client_communicate(self, agent):

		self.socket.connect_with_server()
		
		if not agent.controllers:
			tools = agent.get_tool_ids()
			agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()
		# Let the socket know controller IDs
		self.socket.broadcast_to_server((Constant.CTRL_HOOK, agent.controllers))

		while True:
			# Send to server
			event = p.getKeyboardEvents()
			# Make fake button events
			event[6] = {1: 0}
			self.socket.broadcast_to_server(event)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			for s in signal:
				s = eval(s)
				self._signal_loop(s, agent, control_map, obj_map)

			# time.sleep(0.01)
	# def local_communicate(self, agent, gui=True):

	# 	link_info = agent.get_tool_link_states(-1)
	# 	# Set same number of controllers as number of arms/grippers
	# 	agent.set_virtual_controller(range(len(link_info)))
	# 	self.control_map = agent.create_control_mappings()

	# 	self.pos = [list(i[0]) for i in link_info]
	# 	self.pseudo_event = {0: 0, 3: 0.0}

	# 	while True:

	# 		events = self.server.read_msg()
	# 		self._event_handler(events, agent)
	# 		time.sleep(0.01)

	# 		

class IKeyboard(CtrlInterface):

	def __init__(self, host, remote, task_name):
		# Default settings for camera
		super(IKeyboard, self).__init__(host, remote, task_name)
		self.pos = []

	def client_communicate(self, agent, configs):

		self.socket.connect_with_server()

		# Let the socket know controller IDs
		self.socket.broadcast_to_server(configs)

		self.socket.broadcast_to_server(
			(Constant.CTRL_HOOK, [0, 1])
		)

		if not agent.controllers:
			tools = agent.get_tool_ids()
			agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()
		
		while True:
			# Send to server
			event = p.getKeyboardEvents()
			# Make fake button events
			event[6] = {1: 0}
			self.socket.broadcast_to_server(event)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			
			for s in signal:
				s = eval(s)
				self._signal_loop(s, agent, control_map, obj_map)

			if self.control_id:
				p.resetDebugVisualizerCamera(0.4, 75, -40, 
					p.getBasePositionAndOrientation(self.control_id)[0])

			time.sleep(0.005)

	def server_communicate(self, agent, scene, task, gui=False):
		
		self.socket.connect_with_client()

		tools = agent.get_tool_ids()
		agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()

		end_effector_poses = agent.get_tool_poses(tools)
		self.pos = end_effector_poses[:, 0]
		self.orn = [[0,0,0],[0,0,0]]
		pseudo_event = {0: 0, 3: 0.0, 6: {1: 0}}

		curr_tool_id = control_map[Constant.GRIPPER][pseudo_event[0]]

		# First include entire scene
		self.msg_holder = self._msg_wrapper(curr_tool_id, 
			range(p.getNumBodies()), agent, obj_map)

		while True:
			# if agent.controllers:
			events = self.socket.listen_to_client()
			for event in events:
				e = eval(event)
				return_status = self._event_loop(e, scene, task, agent, gui)
				if return_status > 0:
					# The event dictionary sent
					self._keyboard_event_handler(e, agent, control_map, pseudo_event)
				elif return_status < 0:
					p.disconnect()
					raise KeyboardInterrupt
				else:
					control_map, _ = agent.create_control_mappings()
					end_effector_poses = agent.get_tool_poses(tools)
					self.pos = end_effector_poses[:, 0]
					self.orn = [[0,0,0],[0,0,0]]
			if not gui:
				p.stepSimulation()

			curr_tool_id = control_map[Constant.GRIPPER][pseudo_event[0]]

			flow_lst = agent.arms + agent.grippers
			for i in range(p.getNumBodies()):
				if i not in agent.arms and i not in agent.grippers:
					orig_pos, orig_orn = self.msg_holder[i][:2]
					pos, orn = p.getBasePositionAndOrientation(i)[:2]
					if np.allclose(orig_pos, pos, rtol=3e-5) and np.allclose(orig_orn, 
						orn, rtol=3e-5):
						continue
					flow_lst.append(i)

			self.socket.broadcast_to_client(self._msg_wrapper(curr_tool_id, 
				flow_lst, agent, obj_map))

	def local_communicate(self, agent, gui=True):
		
		tools = agent.get_tool_ids()

		end_effector_poses = agent.get_tool_poses(tools)
		self.pos = end_effector_poses[:, 0]
		self.orn = [[0, 0, 0], [0, 0, 0]]
		# Set same number of controllers as number of arms/grippers
		agent.set_virtual_controller(range(len(tools)))
		control_map, _ = agent.create_control_mappings()
		pseudo_event = {0: 0, 3: 0.0}
		done, success = False, False
		while not done:
			events = p.getKeyboardEvents()
			self._keyboard_event_handler(events, agent, control_map, pseudo_event)	
			if not gui:
				p.stepSimulation()
			time.sleep(0.01)
			done, success = self._checker.check_done()
		return success

	def _keyboard_event_handler(self, events, agent, control_map, pseudo_event):
		
		for e in (events):
			if e not in Constant.HOT_KEYS:
				continue
			if not agent.solo:
				if Key.ONE(e) and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 0
				elif Key.TWO(e) and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 1

			pseudo_event[2] = (0, 1, 0, 0)
			pseudo_event[6] = {32: 1, 33: 0, 1: 0}

			# Position control
			if Key.X in events and (events[Key.X] == p.KEY_IS_DOWN):
				if Key.UP(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][0] += 0.01
				elif Key.DOWN(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][0] -= 0.01

			if Key.Y in events and (events[Key.Y] == p.KEY_IS_DOWN):
				if Key.LEFT(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][1] -= 0.01
				if Key.RIGHT(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][1] += 0.01	

			if Key.Z in events and (events[Key.Z] == p.KEY_IS_DOWN):
				if Key.UP(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][2] += 0.01 		
				if Key.DOWN(e) and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][2] -= 0.01

			# Orientation control
			if Key.S in events and (events[Key.S] == p.KEY_IS_DOWN):
				if Key.UP(e) and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][2] -= 0.01
				if Key.DOWN(e) and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][2] += 0.01
				if Key.LEFT(e) and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][0] -= 0.01
				if Key.RIGHT(e) and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][0] += 0.01
				if Key.R in events and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][1] -= 0.01
				if Key.CR in events and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][1] += 0.01

			# Gripper control
			if Key.G in events and (events[Key.G] == p.KEY_IS_DOWN):
				
				# Using binary grippers for keyboard control
				if agent.close_grip:
					# This for binary robot gripper
					agent.release(control_map[Constant.GRIPPER][pseudo_event[0]])
					# This for binary pr2 
					pseudo_event[3] = 0.0
				else:
					agent.grip(control_map[Constant.GRIPPER][pseudo_event[0]])
					pseudo_event[3] = 1.0

			# Update position
			pseudo_event[1] = self.pos[pseudo_event[0]]

			# Update orientation with limits
			if hasattr(agent, 'UPPER_LIMITS') or hasattr(agent, 'LOWER_LIMITS'):
				self.orn = [np.arcsin(np.sin(rad)) for rad in self.orn]

			if not agent.FIX:
				pseudo_event[2] = p.getQuaternionFromEuler(self.orn[pseudo_event[0]])

			# If disengaged, reset position
			try:
				if agent.control(pseudo_event, control_map) < 0:
					poses = agent.get_tool_poses(agent.get_tool_ids())
					self.pos = poses[:, 0]

			except handler.IllegalOperation as e:
				if self.socket:
					handler.illegal_operation_handler(e, self.socket)
				self.orn = [[0,0,0],[0,0,0]]
				continue


class IVR(CtrlInterface):

	def __init__(self, host, remote, task_name):
		# Default settings for camera
		super(IVR, self).__init__(host, remote, task_name)

	def client_communicate(self, agent, configs):

		self.socket.connect_with_server()
		self.socket.broadcast_to_server(configs)

		control_map, obj_map = agent.create_control_mappings()
		
		while True:
			# Let the socket know controller IDs
			self.socket.broadcast_to_server(
				(Constant.CTRL_HOOK, [e[0] for e in p.getVREvents()])
			)
			# Send to server
			events = p.getVREvents()
			for event in (events):
				self.socket.broadcast_to_server(event)


				if event[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
					print('close')

				if event[6][33] & p.VR_BUTTON_WAS_RELEASED:
					print('release')

			signal = self.socket.listen_to_server()




			for s in signal:
				s = eval(s)
				self._signal_loop(s, agent, control_map, obj_map)

			if self.control_id:
				p.resetDebugVisualizerCamera(0.4, 75, -40, 
					p.getBasePositionAndOrientation(self.control_id)[0])
			time.sleep(0.001)

	def server_communicate(self, agent, scene, task, gui=False):

		self.socket.connect_with_client()

		# First get the controller IDs
		while not agent.controllers:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				if isinstance(event, tuple):
					if event[0] is Constant.CTRL_HOOK and event[1]:
						agent.set_virtual_controller(event[1])
						control_map, obj_map = agent.create_control_mappings()

		skip_flag = agent.redundant_control()

		# First include entire scene
		self.msg_holder = self._msg_wrapper(None, range(p.getNumBodies()), agent, obj_map)

		while True:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				return_status = self._event_loop(event, scene, task, 
					agent, gui, skip=skip_flag)
				if return_status > 0:
					try:
						agent.control(event, control_map)
					except handler.IllegalOperation as e:
						handler.illegal_operation_handler(e, self.socket)
						continue
				if return_status < 0:
					p.disconnect()
					raise KeyboardInterrupt
			if not gui:
				p.stepSimulation()


			flow_lst = agent.arms + agent.grippers
			for i in range(p.getNumBodies()):
				if i not in agent.arms and i not in agent.grippers:
					orig_pos, orig_orn = self.msg_holder[i][:2]
					pos, orn = p.getBasePositionAndOrientation(i)[:2]
					if np.allclose(orig_pos, pos, rtol=3e-5) and np.allclose(orig_orn, orn, rtol=3e-5):
						continue
					flow_lst.append(i)

			self.socket.broadcast_to_client(self._msg_wrapper(None, flow_lst, agent, obj_map))

	def local_communicate(self, agent, gui=True):
		control_map, _ = agent.create_control_mappings()
		done, success = False, False
		while not done:
			events = p.getVREvents()
			skip_flag = agent.redundant_control()
			for event in (events):
				try:
					if skip_flag:
						if event[0] == agent.controllers[1]:
							break
						agent.control(event, control_map)
					else:
						agent.control(event, control_map)
				except handler.IllegalOperation:
					continue
			if not gui:
				p.stepSimulation()
			done, success = self._checker.check_done()
		return success

	






