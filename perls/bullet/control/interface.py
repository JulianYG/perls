from bullet.control.hub import *

class CtrlInterface(object):
	"""
	In charge of direct control with pybullet node. Can be either 
	direct VR / keyboard, etc. control on pybullet, or be a hub 
	that connects inputs from elsewhere such as ROS / openAI to 
	the pybullet node.
	"""
	def __init__(self, host, remote):
		self.remote = remote
		self.server = host

	def start_remote_ctrl(self):
		self.remote = True

	def stop_remote_ctrl(self):
		self.remote = False

	def communicate(self, model):
		if self.remote:
			self._remote_comm(model)
		else:
			self._local_comm(model)

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
									p.changeConstraint(control_map[CONSTRAINT][obj_map[GRIPPER][obj]], 
										pose[0], pose[1], maxForce=model.MAX_FORCE)
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

	def close_socket(self):
		if self.remote:
			self.server.close()

	def _remote_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	

