import time, os
import pybullet as p
from bullet.simulators.simulator import Simulator
from os.path import join as pjoin
import struct

class KeyboardSimulator(Simulator):

	def __init__(self, model):
		# Default settings for camera
		super(KeyboardSimulator, self).__init__(model)

	def load_task(self, task, flag):
		if not self._model.reset(flag, 0):
			raise Exception('Cannot create pybullet GUI instance. Please try again.')
		self._model.load_task(task)
		self.link_info = self._model.get_link_info(-1)

		# Set same number of controllers as number of arms/grippers
		self._model.set_virtual_controller(range(len(self.link_info)))
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
			pos = [list(i[0]) for i in self.link_info]

			pseudo_event = {0: 0}
			while True:
				events = p.getKeyboardEvents()

				for e in events:
					if not self._model.solo:
						if e == 49 and (events[e] == p.KEY_IS_DOWN):
							pseudo_event[0] = 0
						elif e == 50 and (events[e] == p.KEY_IS_DOWN):
							pseudo_event[0] = 1

					# eef_pos_x, eef_pos_y, eef_pos_z = pos[pseudo_event[0]]
					# Can add orn too: self.link_info[ps_e[0]][1]

					pseudo_event[2] = (0, 1, 0, 0)
					pseudo_event[6] = {32: 1, 33: 0, 1: 0}

					if 120 in events and (events[120] == p.KEY_IS_DOWN):
						if e == 65297 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][0] += 0.01
						
						elif e == 65298 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][0] -= 0.01

					if 121 in events and (events[121] == p.KEY_IS_DOWN):
						if e == 65296 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][1] += 0.01
						if e == 65295 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][1] -= 0.01	

					if 122 in events and (events[122] == p.KEY_IS_DOWN):
						if e == 65297 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][2] += 0.01 		
						if e == 65298 and (events[e] == p.KEY_IS_DOWN):
							pos[pseudo_event[0]][2] -= 0.01

					# Add rotation

					#TODO: add gripper control, etc
					pseudo_event[1] = pos[pseudo_event[0]]

					# If disengaged, reset position
					if self._model.control(pseudo_event, self._control_map)	< 0:
						pos = [list(i[0]) for i in self._model.get_link_info(-1)]			
				
				time.sleep(0.01)			

		except KeyboardInterrupt:
			self.quit(logIds)

