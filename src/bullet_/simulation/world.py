import pybullet as p
import numpy as np

from simulation.utils.enum import *
from simulation.utils.helpers import *

import csv

class World(object):
	"""
	The basic scene setup in VR 
	"""
	def __init__(self):
		"""
		Other subclasses may re-implement the constructor
		"""		
		self.controllers = None
		self.default_obj_cnt = 0

		self.hand = False
		self.grippers = []
		self.constraints = []
		self.arms = []
		self.loaded_obj = []
		self.VR_HAND_ID = None

		self.solo = None
		self.MAX_FORCE = 500
		self.name_dic = dict()

	def init_control(self):
		"""
		Load task for both recording and replay
		"""
		self.controllers = [e[0] for e in p.getVREvents()]
		return 1

	def setup_scene(self, scene, task, gui=True):
		self.init_control()
		self._load_env(scene)
		self.default_obj_cnt = p.getNumBodies()
		self._load_tools(self.positions)
		self._load_task(task)
		self.loaded_obj = range(self.default_obj_cnt, p.getNumBodies())
		p.setGravity(0, 0, -9.81)
		if not gui:
			for _ in range(100):
				p.stepSimulation()

	def mark(self, text, position, color=[255,0,0], font_size=8, time=10):
		try:
			p.addUserDebugText(text, position, color, font_size, time)
		except Exception:
			print(text)

	def write_body_info(self, file):
		with open(file, 'w') as f:
			writer = csv.writer(f)
			for k, v in self.name_dic.items():
				writer.writerow([k, v])

	def get_loaded_obj(self):
		return self.loaded_obj

	def _load_env(self, env):
		for obj_pose in env:
			if len(obj_pose) == 3:
				ob = p.loadSDF(obj_pose[0])[0]
				p.resetBasePositionAndOrientation(ob, 
					obj_pose[1], obj_pose[2])
			else:
				ob = p.loadURDF(
					obj_pose[0], obj_pose[1], 
					obj_pose[2], useFixedBase=obj_pose[3]
				)
			self.name_dic[ob] = p.getBodyInfo(ob)[1]

	def _load_task(self, task):
		for obj_pose in task:
			if len(obj_pose) == 1:
				ob = p.loadSDF(obj_pose[0])
			else:
				ob = p.loadURDF(*obj_pose)
			self.name_dic[ob] = p.getBodyInfo(ob)[1]



	