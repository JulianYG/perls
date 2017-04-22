import pybullet as p
import time
from bullet.util import _RESET_HOOK, _SHUTDOWN_HOOK, _START_HOOK
from bullet.util import POS_CTRL, VEL_CTRL

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

		model.reset(0, vr)
		model.setup_scene(task)

		# Reset server side simulation
		self.server.broadcast_msg(_RESET_HOOK)

		control_map, obj_map = model.create_control_mappings()

		def _cmd_handler(msg):

			packet = msg['data']
			if isinstance(packet, str) or isinstance(packet, bytes):
				data = eval(packet)
				if data == _SHUTDOWN_HOOK:
					raise SystemExit('Server invokes shut down')
				elif data == _START_HOOK:
					print('Server is online')
					# model.reset(0, vr)
					# model.setup_scene(task)
					# p.setRealTimeSimulation(0)
				else:
					for obj, pose in data.items():
						if (obj not in model.grippers) and (obj not in model.arms):
							p.resetBasePositionAndOrientation(obj, pose[0], pose[1])
						else:
							if obj in model.grippers:
								# Check if this is pr2 instance by checking arms (pr2 does not contain arms)
								if not model.arms:
									# Change the gripper constraint if obj is pr2 gripper (move it)
									p.changeConstraint(control_map[CONSTRAINT][obj_map[GRIPPER][obj]], 
										pose[0], pose[1], maxForce=model.MAX_FORCE)

								# If robot arm instance, just set gripper close/release
								# The same thing for pr2 gripper
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
				self.server.broadcast_msg(e)
			time.sleep(0.001)

	def close_socket(self):
		if self.remote:
			self.server.close()

	def _remote_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	

