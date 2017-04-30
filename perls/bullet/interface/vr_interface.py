from bullet.interface.core import CtrlInterface
import pybullet as p
import time, sys
import redis
from numpy import array
from bullet.utils.classes import *
from bullet.utils.enum import *

class IVR(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def client_communicate(self, agent):

		self.socket.connect_with_server()
		control_map, obj_map = agent.create_control_mappings()

		# Let the socket know controller IDs
		self.socket.broadcast_to_server((CTRL_HOOK, agent.controllers))
		while True:
			# Send to server
			events = p.getVREvents()
			for e in (events):
				self.socket.broadcast_to_server(e)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			for s in signal:
				s = eval(s)
				if s is SHUTDOWN_HOOK:
					raise KeyboardInterrupt('Server invokes shutdown')
					continue
				if s is START_HOOK:
					print('Server is online')
					continue
				if isinstance(s, WARNING_HOOK):
					agent.mark(*WARNING_HOOK.content)
				self._render_from_signal(agent, control_map, obj_map, s)
			time.sleep(0.001)

	def server_communicate(self, agent, scene, task, gui=True):

		self.socket.connect_with_client()

		# First get the controller IDs
		while not agent.controllers:
			events = self.socket.listen_to_client()
			for e in events:
				e = eval(e)
				if isinstance(e, tuple):
					if e[0] is CTRL_HOOK:
						agent.set_virtual_controller(e[1])
						control_map, obj_map = agent.create_control_mappings()

		skip_flag = agent.redundant_control()

		while True:
			events = self.socket.listen_to_client()
			for e in events:
				e = eval(e)
				if skip_flag:
					if e[0] == agent.controllers[1]:
						break
				# Hook handlers
				if e is RESET_HOOK:
					print('VR Client connected. Initializing reset...')
					p.setInternalSimFlags(0)
					p.resetSimulation()
					agent.solo = len(agent.arms) == 1 or len(agent.grippers) == 1
					agent.setup_scene(scene, task, gui)
				
				elif e is SHUTDOWN_HOOK:
					print('VR Client quit')
				
				elif isinstance(e, tuple) and len(e) == 2:
					if e[0] is CTRL_HOOK:
						agent.set_virtual_controller(e[1])
						print('Received VR controller IDs: {}'.format(agent.controllers))

				else:
					# Add user interaction for task completion
					# Can add line for mark here
					# so that in saved csv file, we know when one task is complete	
					if (e[self.BTTN][1] & p.VR_BUTTON_WAS_TRIGGERED):
						print('User finished one task.')
						self.socket.broadcast_to_client(
							_WARNING_HOOK([
								'Good job! You completed one piece of task',
								(1.7, 0, 1), (255, 0, 0), 12, 5]
								)
							)
					try:
						agent.control(e, control_map)
					except IllegalOperation as e:
						print('Captured client\'s illegal operation. Notifying client')

						self.socket.broadcast_to_client(
							_WARNING_HOOK([
								'Warning: you are flipping arm link {}'.format(e.link),
								p.getLinkState(arm_id, 6 - e.link)[0], (255, 0, 0), 12, 1.5]
								)
							)
						continue
			if not gui:
				p.stepSimulation()
		

			msg = self._msg_wrapper(agent, obj_map)
			# time1 = time.time()
			self.socket.broadcast_to_client(msg)
			# ts = time.time() - time1
			# dr = sys.getsizeof(msg) / ts
			# print('wrapping and sending at rate {}bps'.format(dr))


	def local_communicate(self, agent, gui=True):
		control_map, _ = agent.create_control_mappings()
		while True:
			events = p.getVREvents()
			skip_flag = agent.redundant_control()
			for e in (events):
				try:
					if skip_flag:
						if e[0] == agent.controllers[1]:
							break
						agent.control(e, control_map)
					else:
						agent.control(e, control_map)
				except IllegalOperation:
					continue
			if not gui:
				p.stepSimulation()

	


