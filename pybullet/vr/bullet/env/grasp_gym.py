import math
from bullet.models.kuka import Kuka
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

	def __init__(self, model, task, reward, observ, step_func):
		self.theta_threshold_radians = 0.01
	    self.x_threshold = 0.24
	    self.model = model
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
		self.model.set_time_step(self.timeStep)
		self.model.reset(0)
		self.model.setup_scene(task)
		self.tools = self.model.get_tool_ids()
		self.force_sensor = self.model.has_force_sensor

	def _step(self, action):
		self.model.step_simulation()

		done = self._step_helper(self.model)

	    # self.state, _ = np.array(self.model.get_tool_states())

	    # x, x_dot = self.state[:, :, :2]
	    # if self.force_sensor:
	    # 	force = self.state[:, :, 2]

	    # # self.model.set_tool_pos(self.tool_ids, positions)
	    # self.model.set_tool_states(self.tool_ids, action + x_dot, 'vel')

	    # done = x < -self.x_threshold \
	    #     or x > self.x_threshold \
	    #     or theta < -self.x_dot_threshold \
	    #     or theta > self.x_dot_threshold

	    # eef_pos, eef_orn, eef_vel = self.model.get_tool_link_states(6)

	    reward = self._reward(self.model) #1.0 / np.sum((np.array(eef_pos) - self.goal) ** 2)
	    observe = self._observe(self.model)
	    return observe, reward, done, {}	# info


