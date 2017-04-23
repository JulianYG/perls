from bullet.control.interface import CtrlInterface
import pybullet as p
import time
import redis
from bullet.util import ARM, GRIPPER
from bullet.util import _RESET_HOOK, _SHUTDOWN_HOOK, _START_HOOK

class IVR(CtrlInterface):

	# TODO: reset signal, pass controller IDs, add configuration file system

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def _remote_comm(self, model):

		model.set_virtual_controller([3, 4])
		self.socket.connect_with_client()
		control_map, obj_map = model.create_control_mappings()

		while True:
			if model.controllers:
				events = self.socket.listen_to_client()

				if events == _RESET_HOOK:
					p.resetSimulation()
					continue

				if events == _SHUTDOWN_HOOK:
					print('VR Client quit')
					continue

				# if isinstance(events, list):
				# 	model.set_virtual_controller(events)

				skip_flag = model.redundant_control()
				for e in (events):
					if skip_flag:
						if e[0] == model.controllers[1]:
							break
						model.control(e, control_map)
					else:
						model.control(e, control_map)

				msg = {}
				for ID in range(p.getNumBodies()):
					msg[ID] = list(p.getBasePositionAndOrientation(ID)[:2])
					# If containing arms, send back arm and gripper info
					# since each arm must have one gripper
					if model.arms:
						if ID in obj_map[ARM] or ID in obj_map[GRIPPER]:
							msg[ID] += [model.get_tool_joint_states(ID)]
					if model.grippers:
						if ID in obj_map[GRIPPER]:
							msg[ID] += [model.get_tool_joint_states(ID)]

				self.socket.broadcast_to_client(msg)

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

	


