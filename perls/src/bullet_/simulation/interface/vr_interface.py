import pybullet as p
import time, sys
import redis
from numpy import array

from simulation.interface.core import CtrlInterface
from simulation.utils.classes import *
from simulation.utils.enum import *

class IVR(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def client_communicate(self):

		self.socket.connect_with_server()

		# Let the socket know controller IDs
		self.socket.broadcast_to_server(
			(CTRL_HOOK, [e[0] for e in p.getVREvents()])
		)
		while True:
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
					if event[0] is CTRL_HOOK:
						agent.set_virtual_controller(event[1])
						control_map, obj_map = agent.create_control_mappings()

		skip_flag = agent.redundant_control()

		while True:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				if self._event_loop(event, scene, task, 
					agent, gui, skip=skip_flag) > 0:
					try:
						agent.control(event, control_map)
					except IllegalOperation as e:
						illegal_operation_handler(e, self.socket)
						continue
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

	


