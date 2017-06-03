import pybullet as p
import time, sys
from simulation.utils.classes import *
from simulation.utils.enum import *
from numpy import array
import numpy as np

class CtrlInterface(object):
	"""
	In charge of direct control with pybullet node. Can be either 
	direct VR / keyboard, etc. control on pybullet, or be a hub 
	that connects inputs from elsewhere such as ROS / openAI to 
	the pybullet node.
	"""
	def __init__(self, host, remote):
		self.remote = remote
		self.socket = host
		self.BTTN = 6
		self.ORTN = 2
		self.POSN = 1
		self.AAX = 3

	def start_remote_ctrl(self):
		self.remote = True

	def stop_remote_ctrl(self):
		self.remote = False

	def client_communicate(self, agent, task):
		raise NotImplementedError('Each interface must re-implement this method.')

	def server_communicate(self, agent, scene, task, gui=True):
		raise NotImplementedError('Each interface must re-implement this method.')

	def local_communicate(self, agent, gui=True):
		raise NotImplementedError('Each interface must re-implement this method.')

	def communicate(self, agent, scene, task, gui):
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
		if event is RESET_HOOK:
			print('VR Client connected. Initializing reset...')
			p.setInternalSimFlags(0)
			p.resetSimulation()
			agent.constraints = []
			agent.setup_scene(scene, task, gui)
			agent.solo = len(agent.arms) == 1 or len(agent.grippers) == 1
			return 0 
		elif event is SHUTDOWN_HOOK:
			print('VR Client quit')
			return -1
		elif isinstance(event, tuple) and len(event) == 2:
			if event[0] is CTRL_HOOK and event[1]:
				agent.set_virtual_controller(event[1])
				print('Received controller IDs: {}'.format(agent.controllers))
			return 0
		else:
			# Add user interaction for task completion
			# Can add line for mark here so that in saved csv file, 
			# we know when one task is complete	
			if (event[self.BTTN][1] & p.VR_BUTTON_WAS_TRIGGERED):
				task_monitor_handler(self.socket)
			return 1


class ICmd(CtrlInterface):

	def __init__(self, host, remote):
		super(ICmd, self).__init__(host, remote)

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

				except IllegalOperation as e:
					illegal_operation_handler(e, self.socket)
					continue
			if not gui:
				p.stepSimulation()

	def client_communicate(self):

		self.socket.connect_with_server()
		
		if not agent.controllers:
			tools = agent.get_tool_ids()
			agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()
		# Let the socket know controller IDs
		self.socket.broadcast_to_server((CTRL_HOOK, agent.controllers))

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

			time.sleep(0.01)
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

	def __init__(self, host, remote):
		# Default settings for camera
		super(IKeyboard, self).__init__(host, remote)
		self.pos = []

	def client_communicate(self):

		self.socket.connect_with_server()
	
		p.connect(p.GUI)
		# Let the socket know controller IDs
		self.socket.broadcast_to_server(
			(CTRL_HOOK, [0, 1])
		)
		
		while True:
			# Send to server
			event = p.getKeyboardEvents()
			# Make fake button events
			event[6] = {1: 0}
			self.socket.broadcast_to_server(event)
			time.sleep(0.01)

	def server_communicate(self, agent, scene, task, gui=False):
		
		self.socket.connect_with_client()

		tools = agent.get_tool_ids()
		agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()

		end_effector_poses = agent.get_tool_poses(tools)
		self.pos = end_effector_poses[:, 0]
		self.orn = [[0,0,0],[0,0,0]]
		pseudo_event = {0: 0, 3: 0.0, 6: {1: 0}}

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

			# self.socket.broadcast_to_client(self._msg_wrapper(agent, obj_map))

	def local_communicate(self, agent, gui=True):
		
		tools = agent.get_tool_ids()

		end_effector_poses = agent.get_tool_poses(tools)
		self.pos = end_effector_poses[:, 0]
		self.orn = [[0, 0, 0], [0, 0, 0]]
		# Set same number of controllers as number of arms/grippers
		agent.set_virtual_controller(range(len(tools)))
		control_map, _ = agent.create_control_mappings()
		pseudo_event = {0: 0, 3: 0.0}

		while True:
			events = p.getKeyboardEvents()
			self._keyboard_event_handler(events, agent, control_map, pseudo_event)	
			if not gui:
				p.stepSimulation()
			time.sleep(0.01)

	def _keyboard_event_handler(self, events, agent, control_map, pseudo_event):

		for e in (events):
			if e not in HOT_KEYS:
				continue
			if not agent.solo:
				if e == 49 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 0
				elif e == 50 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 1

			pseudo_event[2] = (0, 1, 0, 0)
			pseudo_event[6] = {32: 1, 33: 0, 1: 0}

			# Keyboard mappings:
			# 1: 49  2: 50
			# x: 120  y: 121 z: 122
			# up: 65298 down: 65297 left: 65295 right: 65296
			# c: 99 r: 114 o: 111

			# Position control
			if 120 in events and (events[120] == p.KEY_IS_DOWN):
				if e == 65298 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][0] += 0.01
				elif e == 65297 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][0] -= 0.01

			if 121 in events and (events[121] == p.KEY_IS_DOWN):
				if e == 65296 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][1] += 0.01
				if e == 65295 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][1] -= 0.01	

			if 122 in events and (events[122] == p.KEY_IS_DOWN):
				if e == 65297 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][2] += 0.01 		
				if e == 65298 and (events[e] == p.KEY_IS_DOWN):
					self.pos[pseudo_event[0]][2] -= 0.01

			# Orientation control
			if 111 in events and (events[111] == p.KEY_IS_DOWN):
				if e == 65297 and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][1] -= 0.005
				if e == 65298 and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][1] += 0.005
				if e == 65295 and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][0] -= 0.005
				if e == 65296 and (events[e] == p.KEY_IS_DOWN):
					self.orn[pseudo_event[0]][0] += 0.005

			# Gripper control
			if 99 in events and (events[99] == p.KEY_IS_DOWN):
				# Using binary grippers for keyboard control
				agent.grip(control_map[GRIPPER][pseudo_event[0]])
				pseudo_event[3] = 1.0

			if 114 in events and (events[114] == p.KEY_IS_DOWN):
				# This for binary robot gripper
				agent.release(control_map[GRIPPER][pseudo_event[0]])
				# This for binary pr2 
				pseudo_event[3] = 0.0

			# Update position
			pseudo_event[1] = self.pos[pseudo_event[0]]
			# Update orientation with limits

			self.orn = [np.arcsin(np.sin(rad)) for rad in self.orn]
			if not agent.FIX:
				pseudo_event[2] = p.getQuaternionFromEuler(self.orn[pseudo_event[0]])

			# If disengaged, reset position
			try:
				if agent.control(pseudo_event, control_map) < 0:
					poses = agent.get_tool_poses(agent.get_tool_ids())
					self.pos = poses[:, 0]

			except IllegalOperation as e:
				if self.socket:
					illegal_operation_handler(e, self.socket)
				self.orn = [[0,0,0],[0,0,0]]
				continue


class IVR(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def client_communicate(self):

		self.socket.connect_with_server()

		p.connect(p.SHARED_MEMORY)
		
		while True:
			# Let the socket know controller IDs
			self.socket.broadcast_to_server(
				(CTRL_HOOK, [e[0] for e in p.getVREvents()])
			)
			# Send to server
			events = p.getVREvents()
			for event in (events):
				self.socket.broadcast_to_server(event)
			time.sleep(0.001)

	def server_communicate(self, agent, scene, task, gui=False):

		self.socket.connect_with_client()

		# First get the controller IDs
		while not agent.controllers:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				if isinstance(event, tuple):
					if event[0] is CTRL_HOOK and event[1]:
						agent.set_virtual_controller(event[1])
						control_map, obj_map = agent.create_control_mappings()

		skip_flag = agent.redundant_control()

		while True:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				return_status = self._event_loop(event, scene, task, 
					agent, gui, skip=skip_flag)
				if return_status > 0:
					try:
						agent.control(event, control_map)
					except IllegalOperation as e:
						illegal_operation_handler(e, self.socket)
						continue
				if return_status < 0:
					p.disconnect()
					raise KeyboardInterrupt
			if not gui:
				p.stepSimulation()

	def local_communicate(self, agent, gui=True):
		control_map, _ = agent.create_control_mappings()
		while True:
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
				except IllegalOperation:
					continue
			if not gui:
				p.stepSimulation()

	






