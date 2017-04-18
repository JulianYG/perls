from bullet.control.hub.hub import Hub

class GymHub(Hub):

	def __init__(self, terminal):
		super(GymHub, self).__init__(terminal)

	def broadcast_msg(self, msg):
		raise NotImplementedError('Each messager should re-implement this method')
		
	def read_msg(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def connect(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def close(self):
		raise NotImplementedError('Each messager should re-implement this method')
		