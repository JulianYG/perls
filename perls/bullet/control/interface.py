import pybullet as p
import time
from bullet.util import _RESET_HOOK, _SHUTDOWN_HOOK, _START_HOOK, _CTRL_HOOK
from bullet.util import POS_CTRL, VEL_CTRL
from bullet.util import CONSTRAINT, GRIPPER, ARM

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
			self.server_communicate(model)
		else:
			self.local_communicate(model)

	def client_communicate(self, model, task):
		raise NotImplementedError('Each interface must re-implement this method.')

	def close(self):
		if self.remote:
			self.socket.disconnect()

	def _render_from_signal(self, model, control_map, obj_map, signal):

		for obj, pose in signal.items():
			if (obj not in model.grippers) and (obj not in model.arms):
				p.resetBasePositionAndOrientation(obj, pose[0], pose[1])
			else:
				# Check if this is pr2 instance by checking arms (pr2 does not contain arms)
				if obj in model.grippers:
					# Change the gripper constraint if obj is pr2 gripper (move it)
					if not model.arms:
						p.changeConstraint(control_map[CONSTRAINT][obj_map[GRIPPER][obj]], 
                        	pose[0], pose[1], maxForce=model.MAX_FORCE)

					# If robot arm instance, just set gripper close/release
                    # The same thing for pr2 gripper
					model.set_tool_joint_states([obj], [pose[2]], POS_CTRL)

				if obj in model.arms:
					model.set_tool_joint_states([obj], [pose[2]], POS_CTRL)

	def _msg_wrapper(self, model, obj_map, ctrl=POS_CTRL):

		# TODO: reserve case when force sensors are enabled (3 columns joint matrix)
		msg = {}
		for ID in range(p.getNumBodies()):
			msg[ID] = list(p.getBasePositionAndOrientation(ID)[:2])
			# If containing arms, send back arm and gripper info
			# since each arm must have one gripper
			if model.arms:
				# Use list to convert np.array cuz eval does not recognize array
				if ID in obj_map[ARM] or ID in obj_map[GRIPPER]:
					msg[ID] += [list(model.get_tool_joint_states(ID)[0][:, ctrl])]
			if model.grippers:
				if ID in obj_map[GRIPPER]:
					msg[ID] += [list(model.get_tool_joint_states(ID)[0][:, ctrl])]
		return msg

	def _remote_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')



	

