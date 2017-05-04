import pybullet as p
import time, sys
from simulation.utils.classes import *
from simulation.utils.enum import *
from numpy import array

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
			return 0
		elif isinstance(event, tuple) and len(event) == 2:
			if event[0] is CTRL_HOOK:
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

	def _signal_wrapper(self, agent, obj_map, ctrl=POS_CTRL):
		"""
		TODO: send optical flow for streaming service
		"""
		pass


	