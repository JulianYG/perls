from bullet.control.interface import CtrlInterface
import pybullet as p
import time
import redis
from bullet.util import *

class IVR(CtrlInterface):

	# TODO: reset signal, pass controller IDs, add configuration file system

	def __init__(self, host, remote):
		# Default settings for camera
		super(IVR, self).__init__(host, remote)

	def event_callback(self, model, task, vr):

		# if not self.server.connected:
		# 	self.server.connect(model)

		model.reset(0, vr)
		model.setup_scene(task)

		control_map, obj_map = model.create_control_mappings()

		def _cmd_handler(msg):

			packet = msg['data']
			if isinstance(packet, str) or isinstance(packet, bytes):
				data = eval(packet)
				if data == 0:
					# raise SystemExit('Remote client invoke shut down')
					pass
				elif data == 1:
					# print('Remote client invode reset')
					# model.reset(0, vr)
					# p.setRealTimeSimulation(0)
					pass
				else:
					for obj, pose in data.items():
						if (obj not in model.grippers) or (obj not in  model.arms):
							p.resetBasePositionAndOrientation(obj, pose[0], pose[1])
						else:
							if obj in model.grippers:
								# Check if this is pr2 instance by checking arms (pr2 does not contain arms)
								if model.arms:
									# If robot arm instance, just set gripper close/release
									model.set_tool_states([obj], [pose[2]], POS_CTRL)
								else:
									# Change the gripper constraint if obj is pr2 gripper (move it)
									p.changeConstraint(control_map[CONSTRAINT][obj_map[GRIPPER][obj]], pose[0], pose[1], maxForce=500)
									model.set_tool_states([obj], [pose[2]], POS_CTRL)
							
							if obj in model.arms:
								model.set_tool_states([obj], [pose[2]], POS_CTRL)

				# TODO: Handle constraints of grippers; 
				# Handle joint states

		self.server.pubsub.subscribe(**{'client_channel': _cmd_handler})
		self.server.pubsub.run_in_thread(sleep_time=0.001)

		# Ctrl, running in different thread?
		while True:
			events = p.getVREvents()
			for e in (events):
				self.server.terminal.publish('server_channel', e)
			time.sleep(0.001)

	def _remote_comm(self, model):
		tool = model.get_tool_ids()
		model.set_virtual_controller([3, 4])
		
		self.server.connect(model)

		control_map, obj_map = model.create_control_mappings()

		while True:
			if model.controllers:
				events = self.server.read_msg()

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

					# TODO: change hardcoded 1 to something generalized
					msg[ID] = list(p.getBasePositionAndOrientation(ID)[:2])

					# If containing arms, send back arm and gripper info
					# since each arm must have one gripper
					if model.arms:
						if ID in obj_map[ARM] or ID in obj_map[GRIPPER]:
							msg[ID] += [model.get_tool_joint_states(ID)]
					if model.grippers:
						if ID in obj_map[GRIPPER]:
							msg[ID] += [model.get_tool_joint_states(ID)]

				self.server.broadcast_msg(msg)

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

	


