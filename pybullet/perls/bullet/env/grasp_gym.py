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

	def __init__(self, agent, task, reward, observ, step_func):
	    self.agent = agent
	    self.goal = np.array(final_pos)
	    self._seed()
	    self._step_helper = step_func
	    self._observe = observ
	    self._reward = reward
 
	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def _configure(self, display=None):
		self.display = display

	def _reset(self):
		self.timeStep = 0.01
		self.agent.set_time_step(self.timeStep)
		self.agent.reset(0)
		self.agent.setup_scene(task)
		self.tools = self.agent.get_tool_ids()
		self.force_sensor = self.agent.has_force_sensor
		return self._observe(self.agent)[0]

	def _step(self, action):
		self.agent.step_simulation()
		done = self._step_helper(self.agent)
		reward = self._reward(self.agent)
		observe, info = self._observe(self.agent)
		return observe, reward, done, info

	    # self.state, _ = np.array(self.agent.get_tool_states())

	    # x, x_dot = self.state[:, :, :2]
	    # if self.force_sensor:
	    # 	force = self.state[:, :, 2]

	    # # self.agent.set_tool_pos(self.tool_ids, positions)
	    # self.agent.set_tool_states(self.tool_ids, action + x_dot, 'vel')

	    # done = x < -self.x_threshold \
	    #     or x > self.x_threshold \
	    #     or theta < -self.x_dot_threshold \
	    #     or theta > self.x_dot_threshold

	    # eef_pos, eef_orn, eef_vel = self.agent.get_tool_link_states(6)
	    
