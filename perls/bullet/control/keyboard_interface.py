import time
import pybullet as p
from bullet.control.interface import CtrlInterface
from bullet.util import GRIPPER
from bullet.util import _RESET_HOOK, _SHUTDOWN_HOOK, _START_HOOK, _CTRL_HOOK

class IKeyboard(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IKeyboard, self).__init__(host, remote)
		self.pos = []

	def client_communicate(self, agent, task):

		self.socket.connect_with_server()
		
		if not agent.controllers:
			tool = agent.get_tool_ids()
			agent.set_virtual_controller(range(len(tool)))

		control_map, obj_map = agent.create_control_mappings()
		# Let the socket know controller IDs
		self.socket.broadcast_to_server((_CTRL_HOOK, agent.controllers))

		while True:
			# Send to server
			events = p.getKeyboardEvents()
			self.socket.broadcast_to_server(events)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			for s in signal:
				if s is _SHUTDOWN_HOOK:
					raise KeyboardInterrupt('Server invokes shutdown')
					continue
				if s is _START_HOOK:
					print('Server is online')
					continue
				self._render_from_signal(agent, control_map, obj_map, s)
			time.sleep(0.01)

	def server_communicate(self, agent, task):
		
		self.socket.connect_with_client()

		tool = agent.get_tool_ids()
		agent.set_virtual_controller(range(len(tool)))

		control_map, obj_map = agent.create_control_mappings()

		self.pos = [agent.get_tool_pose(t)[0] for t in tool]

		pseudo_event = {0: 0, 3: 0.0}

		while True:
			# if agent.controllers:
			events = self.socket.listen_to_client()
			for e in events:
				# Hook handlers
				if e is _RESET_HOOK:
					print('VR Client connected. Initializing reset...')
					p.setInternalSimFlags(0)
					p.resetSimulation()
					agent.solo = len(agent.arms) == 1 or len(agent.grippers) == 1
					agent.setup_scene(task)
					self.pos = [agent.get_tool_pose(t)[0] for t in tool]
					continue

				if e is _SHUTDOWN_HOOK:
					print('VR Client quit')
					continue

				# Get the controller signal. Make sure server starts before client
				if isinstance(e, tuple):
					if e[0] is _CTRL_HOOK:
						agent.set_virtual_controller(e[1])
						continue

				# The event dictionary sent
				self._keyboard_event_handler(e, agent, control_map, pseudo_event)

			self.socket.broadcast_to_client(self._msg_wrapper(agent, obj_map))

	def local_communicate(self, agent):
		
		tool = agent.get_tool_ids()
		self.pos = [agent.get_tool_pose(t)[0] for t in tool]

		# Set same number of controllers as number of arms/grippers
		agent.set_virtual_controller(range(len(tool)))
		control_map, _ = agent.create_control_mappings()
		pseudo_event = {0: 0, 3: 0.0}

		while True:
			events = p.getKeyboardEvents()
			self._keyboard_event_handler(events, agent, control_map, pseudo_event)	
			time.sleep(0.01)

	def _keyboard_event_handler(self, events, agent, control_map, pseudo_event):

		for e in (events):
			if not agent.solo:
				if e == 49 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 0
				elif e == 50 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 1

			# eef_pos_x, eef_pos_y, eef_pos_z = self.pos[pseudo_event[0]]
			# Can add orn too: self.link_info[ps_e[0]][1]

			pseudo_event[2] = (0, 1, 0, 0)
			pseudo_event[6] = {32: 1, 33: 0, 1: 0}

			# x: 120  y: 121 z: 122
			# up: 65298 down: 65297 left: 65295 right: 65296
			# c: 99 r: 114
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

			# TODO: Add rotation
			if 99 in events and (events[99] == p.KEY_IS_DOWN):
				agent.grip(control_map[GRIPPER][pseudo_event[0]])
				pseudo_event[3] = 1.0

			if 114 in events and (events[114] == p.KEY_IS_DOWN):
				# This for robot gripper
				agent.release(control_map[GRIPPER][pseudo_event[0]])
				# This for pr2
				pseudo_event[3] = 0.0

			pseudo_event[1] = self.pos[pseudo_event[0]]

			# If disengaged, reset position
			if agent.control(pseudo_event, control_map) < 0:
				self.pos = [agent.get_tool_pose(t)[0] \
					for t in agent.get_tool_ids()]

