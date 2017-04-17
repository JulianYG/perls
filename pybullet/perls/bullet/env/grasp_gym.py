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
 
	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def _render(self, camera_info, mode='rgb_array'):
		self.simulator.set_camera_view(*camera_info)
		if mode == 'rgb_array':
			img_array = self.simulator.snapshot()
		else:
			img_array = self.simulator.snapshot(show=True)
		return img_array

	def _configure(self, display=None):
		self.display = display

	def _reset(self, time_step=0.01):
		if not self.realTimeSimulation:
			self.model.set_time_step(time_step)
		self.model.reset(0)
		self.model.setup_scene(task)
		self.tools = self.model.get_tool_ids()
		self.force_sensor = self.model.has_force_sensor
		return self._step_helper(self.model, None)[0]

	def _step(self, action):
		if not self.realTimeSimulation:
			self.model.step_simulation()
		observation, reward, done, info = self._step_helper(self.model, action)
		return observation, reward, done, info

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
	    
