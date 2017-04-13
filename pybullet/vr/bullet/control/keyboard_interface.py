import time
import pybullet as p
from bullet.control.interface import CtrlInterface

class IKeyboard(CtrlInterface):

	def __init__(self, remote):
		# Default settings for camera
		super(IKeyboard, self).__init__(remote)

	def _remote_comm(self, model):
		
		tool = model.get_tool_ids()
		self.pos = [model.get_tool_pose(t)[0] for t in tool]

		# Set same number of controllers as number of arms/grippers
		model.set_virtual_controller(range(len(tool)))
		self.control_map = model.create_control_mappings()
		self.pseudo_event = {0: 0}

		if self.server.connect() < 0:
			raise Exception('Cannot connect to remote client.')

		while True:
			e = self.server.poll_event()
			self._event_handler(e, model)
			# time.sleep(0.01)

	def _local_comm(self, model):
		
		tool = model.get_tool_ids()
		self.pos = [model.get_tool_pose(t)[0] for t in tool]

		# Set same number of controllers as number of arms/grippers
		model.set_virtual_controller(range(len(tool)))
		self.control_map = model.create_control_mappings()
		self.pseudo_event = {0: 0}

		while True:
			events = p.getKeyboardEvents()
			self._event_handler(events, model)	
			time.sleep(0.01)

	def _event_handler(self, events, model):
		for e in (events):
			if not model.solo:
				if e == 49 and (events[e] == p.KEY_IS_DOWN):
					self.pseudo_event[0] = 0
				elif e == 50 and (events[e] == p.KEY_IS_DOWN):
					self.pseudo_event[0] = 1

			# eef_pos_x, eef_pos_y, eef_pos_z = pos[pseudo_event[0]]
			# Can add orn too: self.link_info[ps_e[0]][1]

			self.pseudo_event[2] = (0, 1, 0, 0)
			self.pseudo_event[6] = {32: 1, 33: 0, 1: 0}

			if 120 in events and (events[120] == p.KEY_IS_DOWN):
				if e == 65298 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][0] += 0.01
				
				elif e == 65297 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][0] -= 0.01

			if 121 in events and (events[121] == p.KEY_IS_DOWN):
				if e == 65296 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][1] += 0.01
				if e == 65295 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][1] -= 0.01	

			if 122 in events and (events[122] == p.KEY_IS_DOWN):
				if e == 65297 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][2] += 0.01 		
				if e == 65298 and (events[e] == p.KEY_IS_DOWN):
					self.pos[self.pseudo_event[0]][2] -= 0.01

			# Add rotation

			#TODO: add gripper control for event[3], use space bar

			self.pseudo_event[3] = 0.0
			self.pseudo_event[1] = self.pos[self.pseudo_event[0]]

			# If disengaged, reset position
			if model.control(self.pseudo_event, self.control_map) < 0:
				self.pos = [model.get_tool_pose(t)[0] \
					for t in model.get_tool_ids()]

