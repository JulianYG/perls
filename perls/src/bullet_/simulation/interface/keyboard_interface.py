import time
import numpy as np
from numpy import array
import pybullet as p

from simulation.interface.core import CtrlInterface
from simulation.utils.enum import *
from simulation.utils.classes import *

class IKeyboard(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IKeyboard, self).__init__(host, remote)
		self.pos = []

	def client_communicate(self):

		self.socket.connect_with_server()
		
		# if not agent.controllers:
		# 	tools = agent.get_tool_ids()
		# 	agent.set_virtual_controller(range(len(tools)))

		# control_map, obj_map = agent.create_control_mappings()


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

			# # Receive and render from server
			# signal = self.socket.listen_to_server()
			# for s in signal:
			# 	s = eval(s)
			# 	self._signal_loop(s, agent, control_map, obj_map)

			time.sleep(0.01)

	def server_communicate(self, agent, scene, task, gui=False):
		
		print(self.socket.connected_with_client)
		self.socket.connect_with_client()

		tools = agent.get_tool_ids()
		agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()

		end_effector_poses = agent.get_tool_poses(tools)
		self.pos = end_effector_poses[:, 0]
		self.orn = [[0,0,0],[0,0,0]]
		pseudo_event = {0: 0, 3: 0.0, 6: {1: 0}}

		print('communicated?')
		while True:
			# if agent.controllers:
			events = self.socket.listen_to_client()
			print(events)
			for event in events:
				e = eval(event)

				if self._event_loop(e, scene, task, agent, gui) > 0:
					# The event dictionary sent
					self._keyboard_event_handler(e, agent, control_map, pseudo_event)
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



