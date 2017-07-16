import gym
from gym import error, spaces, utils
from gym.utils import seeding

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))

from framework.control import Controller


class PerlsEnv(gym.Env):

    """
    Trying to solve InverseKinematics by RL
    """
    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf):

        self._world, self._display, _ = Controller.load_config(conf)
        status = self._display.run()
        self._world.boot(self._display.info['frame'])

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _close(self):
        return NotImplemented

    def _render(self, mode='', close=False):
        return NotImplemented

    def _reset(self):
        return NotImplemented

    def _step(self, action):
        return NotImplemented
