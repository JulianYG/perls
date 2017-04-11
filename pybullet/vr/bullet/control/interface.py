import time, os
from bullet.control.remote import server
from os.path import join as pjoin
import struct

class CtrlInterface(object):

	def __init__(self, remote=False, buffer_size=4096, port=5000):
		self.remote = remote
		if self.remote:
			self.server = server.Server(buffer_size, port)
		else:
			self.server = None

	def remote_ctrl(self, flag):
		self.remote = flag

	def communicate(self, pybullet, model):
		if self.remote:
			self._remote_comm(pybullet, model)
		else:
			self._local_comm(pybullet, model)

	def close_socket(self):
		if self.remote:
			self.server.close()

	def _remote_comm(self, pybullet, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	def _local_comm(self, pybullet, model):
		raise NotImplementedError('Each interface must re-implement this method.')

	

