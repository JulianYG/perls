import time
import pybullet as p
from bullet.control.interface import CtrlInterface
from bullet.util import GRIPPER

class IKeyboard(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IKeyboard, self).__init__(host, remote)
		self.pos = []

	def event_callback(self, model, task):

		self.socket.connect_with_server()

		# Let the socket know controller IDs
		self.socket.broadcast_to_server(model.controllers)

		while True:
			# Send to server
			events = p.getKeyboardEvents()
			self.socket.broadcast_to_server(events)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			self._render_from_signal(model, signal)

	def _remote_comm(self, model):
		
		self.socket.connect_with_client()
		control_map, obj_map = model.create_control_mappings()
		tool = model.get_tool_ids()
		self.pos = [model.get_tool_pose(t)[0] for t in tool]
		pseudo_event = {0: 0}

		## Set same number of controllers as number of arms/grippers
		# model.set_virtual_controller(range(len(tool)))
		while True:
			if model.controllers:
				events = self.socket.listen_to_client()
				for e in events:
					# Hook handlers
					if e is _RESET_HOOK:
						# model.reset?
						continue

					if e is _SHUTDOWN_HOOK:
						print('VR Client quit')
						continue

					# Get the controller signal. Make sure server starts before client
					if isinstance(e, list):
						model.set_virtual_controller(e)
						continue

					# The event dictionary sent
					self._keyboard_event_handler(e, model, control_map, pseudo_event)

				self.socket.broadcast_to_client(self._msg_wrapper(model, obj_map))

	def _local_comm(self, model):
		
		tool = model.get_tool_ids()
		self.pos = [model.get_tool_pose(t)[0] for t in tool]

		# Set same number of controllers as number of arms/grippers
		model.set_virtual_controller(range(len(tool)))
		control_map, _ = model.create_control_mappings()
		pseudo_event = {0: 0}

		while True:
			events = p.getKeyboardEvents()
			self._keyboard_event_handler(events, model, control_map, pseudo_event)	
			time.sleep(0.01)

	def _keyboard_event_handler(self, events, model, control_map, pseudo_event):

		for e in (events):
			if not model.solo:
				if e == 49 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 0
				elif e == 50 and (events[e] == p.KEY_IS_DOWN):
					pseudo_event[0] = 1

			# eef_pos_x, eef_pos_y, eef_pos_z = self.pos[pseudo_event[0]]
			# Can add orn too: self.link_info[ps_e[0]][1]

			pseudo_event[2] = (0, 1, 0, 0)
			pseudo_event[6] = {32: 1, 33: 0, 1: 0}

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

			# Add rotation
			#TODO: add gripper control for event[3], use space bar
			if 99 in events and (events[99] == p.KEY_IS_DOWN):
				model.grip(control_map[GRIPPER][pseudo_event[0]])

			if 114 in events and (events[114] == p.KEY_IS_DOWN):
				model.release(control_map[GRIPPER][pseudo_event[0]])

			pseudo_event[3] = 0.0
			pseudo_event[1] = self.pos[pseudo_event[0]]

			# If disengaged, reset position
			if model.control(pseudo_event, control_map) < 0:
				self.pos = [model.get_tool_pose(t)[0] \
					for t in model.get_tool_ids()]

