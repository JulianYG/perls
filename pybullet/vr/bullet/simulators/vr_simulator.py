import time, os
import pybullet as p
from bullet.simulators.simulator import Simulator
from os.path import join as pjoin
import struct

class VRSimulator(Simulator):

	def __init__(self, model):
		# Default settings for camera
		super(VRSimulator, self).__init__(model)

	def load_task(self, task, flag):
		if not self._model.reset(flag, 1):
			raise Exception('Cannot detect running VR application. Please try again.')
		self._model.load_task(task)
		self._control_map = self._model.create_control_mappings()

	def record(self, file, video=False):
		try:
			logIds = []
			if video:
				# Does logging only need to be called once with SharedMemory? 
				logIds.append(p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 
					pjoin(self.VIDEO_DIR, file + '.mp4')))
			else:
				# Record everything
				logIds.append(p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file)))
				# ctrlLog = p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')
			while True:
				events = p.getVREvents()
				skip_flag = self._model.redundant_control()
				for e in (events):
					if skip_flag:
						if e[0] == self._model.controllers[1]:
							break
						self._model.control(e, self._control_map)
					else:
						self._model.control(e, self._control_map)

		except KeyboardInterrupt:
			self.quit(logIds)

