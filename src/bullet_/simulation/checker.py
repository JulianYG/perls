__package__ = 'bullet_.simulation'

import pybullet as p
import time, sys

from numpy import array
import numpy as np

from .utils import handler
from .utils.misc import Constant, Key

class TaskChecker(object):

	def __init__(self, task_name, name_dic):

		self._task = task_name
		self._body_info = name_dic

	def check_done(self):

		if self._task == 'kitchen':	
				
			table_id = self._body_info['table.urdf']
			white_plate_id = self._body_info['plate_white']
			blue_plate_id = self._body_info['plate_blue']
			green_plate_id = self._body_info['plate_green']

			pan_0 = self._body_info['pan']
			pan_1 = self._body_info['pan10']
			pan_2 = self._body_info['pan11']

			pan_2_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_2)[1])
			flat_orn = [0] * 2
			plate_pass = False

			pan_0_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_0)[1])
			pan_1_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_1)[1])
			pan_2_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_2)[1])
			
			if p.getContactPoints(white_plate_id, green_plate_id) and \
				p.getContactPoints(green_plate_id, blue_plate_id) and \
				p.getContactPoints(table_id, white_plate_id):

				white_plate_pos = p.getBasePositionAndOrientation(white_plate_id)[0]
				green_plate_pos = p.getBasePositionAndOrientation(green_plate_id)[0]
				blue_plate_pos = p.getBasePositionAndOrientation(blue_plate_id)[0]

				if white_plate_pos[2] < green_plate_pos[2] < blue_plate_pos[2]:
					if np.allclose(white_plate_pos[:2], green_plate_pos[:2], rtol=0.01) \
						and np.allclose(green_plate_pos[:2], blue_plate_pos[:2], rtol=0.01):
						plate_pass = True
			if plate_pass:
				if p.getContactPoints(table_id, pan_0) and \
					p.getContactPoints(table_id, pan_1) and \
					p.getContactPoints(table_id, pan_2):

					pan_0_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_0)[1])
					pan_1_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_1)[1])
					pan_2_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(pan_2)[1])
					
					if np.allclose(np.abs(pan_0_orn[:2]), flat_orn, atol=1e-5, rtol=0.) and \
						np.allclose(np.abs(pan_1_orn[:2]), flat_orn, atol=1e-5, rtol=0.) and \
						np.allclose(np.abs(pan_2_orn[:2]), flat_orn, atol=1e-5, rtol=0.):
						print("Task Success")
						return True, True

		return False, False

