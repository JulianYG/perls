from bullet.control.hub import server

class CtrlInterface(object):
	"""
	In charge of direct control with pybullet node. Can be either 
	direct VR / keyboard, etc. control on pybullet, or be a hub 
	that connects inputs from elsewhere such as ROS / openAI to 
	the pybullet node.
	"""
	def __init__(self, remote=False, buffer_size=4096, port=5000):
		self.remote = remote
		if self.remote:
			self.server = server.Server(buffer_size, port)
		else:
			self.server = None

	def remote_ctrl(self, flag):
		self.remote = flag

	def communicate(self, model):
		if self.remote:
			self._remote_comm(model)
		else:
			self._local_comm(model)

	def close_socket(self):
		if self.remote:
			self.server.close()

	def _remote_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	

