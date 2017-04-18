import math
from gym import spaces, Env
import numpy as np
from gym.utils import seeding

class GraspBulletEnv(Env):

	"""
	Trying to solve InverseKinematics by RL
	"""
	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 50
	}

	def __init__(self, simulator, task, step_func, realTime=False):
	    self.simulator = simulator
	    self.model = simulator.model
	    self.realTimeSimulation = realTime
	    self._seed()
	    self._step_helper = step_func
	    # Setup simulator but not running
	    self.simulator.setup(task, 0, 0)

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def _close(self):
		self.simulator.quit()

	def _render(self, mode='rgb_array', close=False):
		if mode == 'rgb_array':
			img_array = self.simulator.snapshot()
		else:
			img_array = self.simulator.snapshot(show=True)
		return img_array

	def _reset(self, time_step=0.01):
		if not self.realTimeSimulation:
			self.model.set_time_step(time_step)
		self.tools = self.model.get_tool_ids()
		return self._step_helper(self.model, None)[0]

	def _step(self, action):
		if not self.realTimeSimulation:
			self.model.step_simulation()
		observation, reward, done, info = self._step_helper(self.model, action)
		return observation, reward, done, info

