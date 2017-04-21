"""
Abstract class that connects pybullet with direct pose command input,
such as openAI gym, or ROS command messages
"""

class Hub(object):

	_SHUT_DOWN_HOOK = 0;
	_RESET_HOOK = 1

	def broadcast_msg(self, msg):
		raise NotImplementedError('Each messager should re-implement this method')
		
	def read_msg(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def connect(self):
		raise NotImplementedError('Each messager should re-implement this method')

	def close(self):
		raise NotImplementedError('Each messager should re-implement this method')
		