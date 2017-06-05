import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from datetime import datetime

class BulletEnv(gym.Env):

	"""
	Trying to solve InverseKinematics by RL
	"""
	metadata = {
		'render.modes': ['human', 'depth', 'segment'],
		'video.frames_per_second': 50
	}

	def __init__(self, simulator, wrapper, time_step, 
		realTime, record, video):

	    self.simulator = simulator
	    self.agent = simulator.tool
	    self.realTimeSimulation = realTime
	    self._seed()
	    self._wrapper = wrapper
	    self.time_step = time_step
	    # Setup simulator but not running
	    self.simulator._setup(0)
	    if record:
	    	self.simulator._record(
	    		datetime.now().strftime('%m-%d-%H-%M'), 
	    		video
	    	)

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def _close(self):
		self.simulator.quit()

	def _render(self, mode='', close=False):
		return self.simulator.snapshot(show=mode)

	def _reset(self):
		if not self.realTimeSimulation:
			self.simulator.set_time_step(self.time_step)
		self.tools = self.agent.get_tool_ids()
		return self._wrapper.reset(self.simulator.scene,
			self.simulator.task, self.simulator.gui)

	def _step(self, action):
		if not self.realTimeSimulation:
			self.simulator.step_simulation()
		return self._wrapper.step(action)
		

