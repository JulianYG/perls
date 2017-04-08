import gym
import math
from bullet.models.kuka import Kuka
from gym import spaces
from gym.utils import seeding

class KukaBulletEnv(gym.Env):

	"""
	Trying to solve InverseKinematics by RL
	"""
	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 50
	}

	def __init__(self):
		# p.connect(p.GUI)
		self.theta_threshold_radians = 0.01
	    self.x_threshold = 0.24
		self._seed()
 
	def _seed(self, seed=None):
    	self.np_random, seed = seeding.np_random(seed)
    	return [seed]

    def _configure(self, display=None):
    	self.display = display

	def _reset(self):
		self.timeStep = 0.01
		self.kuka_env = Kuka([-0.1], fixed=True)
		self.kuka_env.set_time_step(self.timeStep)
		self.kuka_env.reset(0)
		self.state, self.arms = self.kuka_env.get_robot_states()
		self.force_sensor = self.kuka_env.has_force_sensor
		return np.array(self.state)

	def _step(self, action):
		self.kuka_env.step_simulation()
	    self.state, _ = np.array(self.kuka_env.get_robot_states())

	    x, x_dot = self.state[:, :, :2]
	    if self.force_sensor:
	    	force = self.state[:, :, 2]

	    # self.kuka_env.set_robot_pos(self.arms, positions)
	    self.kuka_env.set_robot_pos(self.arms, action + v_dot, 'vel')

	    done = x < -self.x_threshold \
	        or x > self.x_threshold \
	        or theta < -self.x_dot_threshold \
	        or theta > self.x_dot_threshold

	    eef_pos, eef_orn, eef_vel = self.kuka_env.get_link_info(6)

	    reward = 1.0 / np.sum((np.array(eef_pos) - ) ** 2)

	    return self.state, reward, done, {}	# info
