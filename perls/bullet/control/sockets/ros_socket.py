from bullet.control.sockets.sock import Socket 

class RosHub(Socket):

	def __init__(self):
		super(RosHub, self).__init__()

	def broadcast_to_client(self, message):
		raise NotImplementedError('Each messager should re-implement this method')
		
	def broadcast_to_server(self, message):
		raise NotImplementedError('Each messager should re-implement this method')

	def listen_to_client(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def listen_to_server(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def connect_with_client(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def connect_with_server(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def close(self):
		raise NotImplementedError('Each messager should re-implement this method')
		