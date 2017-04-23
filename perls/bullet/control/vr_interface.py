from bullet.control.interface import CtrlInterface
import pybullet as p
import time
import redis
from bullet.util import ARM, GRIPPER
from bullet.util import _RESET_HOOK, _SHUTDOWN_HOOK, _START_HOOK

class IVR(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def event_callback(self, model, task):

		self.socket.connect_with_server()

		# Let the socket know controller IDs
		self.socket.broadcast_to_server(model.controllers)

		while True:
			# Send to server
			events = p.getVREvents()
			for e in (events):
				self.socket.broadcast_to_server(e)
			time.sleep(0.001)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			self._render_from_signal(model, signal)

	def _remote_comm(self, model):

		# model.set_virtual_controller([3, 4])

		self.socket.connect_with_client()
		control_map, obj_map = model.create_control_mappings()
		skip_flag = model.redundant_control()

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

					if isinstance(e, list):
						model.set_virtual_controller(e)
						continue

					if skip_flag:
						if e[0] == model.controllers[1]:
							break
					model.control(e, control_map)

				self.socket.broadcast_to_client(self._msg_wrapper(model, obj_map))

	def _local_comm(self, model):
		control_map, _ = model.create_control_mappings()
		while True:
			events = p.getVREvents()
			skip_flag = model.redundant_control()
			for e in (events):
				if skip_flag:
					if e[0] == model.controllers[1]:
						break
					model.control(e, control_map)
				else:
					model.control(e, control_map)

	


