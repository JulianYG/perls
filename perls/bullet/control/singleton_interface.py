from bullet.control.interface import CtrlInterface

class ISingleton(CtrlInterface):
	"""
	A singleton that does nothing as a placeholder for the 
	simulator in openAI gym pybullet interface. The communication
	goes directly to the model, instead of through interface 
	for comprehensive states control.
	"""
	def server_communicate(self, model, task):
		return

	def local_communicate(self, model):
		return

	def client_communicate(self, model, task):
		return

	

