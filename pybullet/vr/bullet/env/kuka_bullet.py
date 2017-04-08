import gym
import pybullet as p
import math
from gym import spaces
from gym.utils import seeding

class KukaBulletEnv(gym.Env):

	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 50
	}

	def __init__(self):
		p.connect(p.GUI)

	def _reset(self):
		self.



