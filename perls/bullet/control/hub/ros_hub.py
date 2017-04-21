from bullet.control.hub.hub import Hub

class RosHub(Hub):

	def __init__(self):
		super(RosHub, self).__init__()

	def broadcast_msg(self, msg):
		raise NotImplementedError('Each messager should re-implement this method')
		
	def read_msg(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def connect(self, model):
		raise NotImplementedError('Each messager should re-implement this method')

	def close(self):
		raise NotImplementedError('Each messager should re-implement this method')
		