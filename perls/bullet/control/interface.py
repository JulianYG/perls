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
		self.socket = host

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
		
		self.control_map, self.obj_map = model.create_control_mappings()

		# Ctrl, running in different thread?
		while True:
			# Send to server
			events = p.getVREvents()
			for e in (events):
				self.socket.broadcast_to_server(e)
			time.sleep(0.001)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			self._render_from_signal(model, signal)

	def close_socket(self):
		if self.remote:
			self.socket.close()

	def _render_from_signal(self, model, signal):

		for data in signal:
			for obj, pose in data.items():
				if (obj not in model.grippers) and (obj not in model.arms):
					p.resetBasePositionAndOrientation(obj, pose[0], pose[1])
				else:
					# Check if this is pr2 instance by checking arms (pr2 does not contain arms)
					if obj in model.grippers:
				
						# Change the gripper constraint if obj is pr2 gripper (move it)
						if not model.arms:
							p.changeConstraint(self.control_map[CONSTRAINT][self.obj_map[GRIPPER][obj]], 
	                        	pose[0], pose[1], maxForce=model.MAX_FORCE)

						# If robot arm instance, just set gripper close/release
	                    # The same thing for pr2 gripper
						model.set_tool_states([obj], [pose[2]], POS_CTRL)

					if obj in model.arms:

						model.set_tool_states([obj], [pose[2]], POS_CTRL)

	def _remote_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	

