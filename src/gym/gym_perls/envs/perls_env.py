# !/usr/bin/env python

import abc

import gym
import gym.spaces as spaces
from gym.utils import seeding

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '../../../../')))

from lib.control import Controller
from lib.utils import io_util

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class PerlsEnv(gym.Env):
    """
    Construct an gym environment
    """

    Space = spaces

    metadata = {
        'render.modes': ['human', 'rgb', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):
        """
        Initialize the environment
        :param conf_path: the absolute path string to
        the configuration file, default is 'gym-disp.xml'
        """
        conf = io_util.parse_config(conf_path)[0]
        _, self._world, self._display, _ = Controller.load_config(conf, None)
        self._status = self._display.run(None)
        self._world.boot(self._display.info['frame'])

    @abc.abstractproperty
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return NotImplemented

    @property
    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return NotImplemented

    @abc.abstractproperty
    def state(self):
        """
        Get the current state of the environment
        :return: state as defined in state space
        """
        return NotImplemented

    @property
    def done(self):
        """
        Whether the program is finished or not
        :return: Boolean value
        """
        return self._world.check_states()[0]

    @property
    def reward(self):
        """
        Get the reward defined by algorithm
        :return: Some form of reward value, usually float
        """
        return self._world.evaluate()

    def _close(self):
        """
        Shutdown the environment
        :return: None
        """
        self._world.clean_up()
        self._display.close(0)

    def _render(self, mode='rgb', close=False):
        """
        Generate rendered data based on given mode.
        :param mode: string of mode, as specified in metadata
        human: pop up a window and render
        rgb: an rgb array suitable for video
        depth: a depth value array of current env
        seg: segmentation value array
        :param close: close all open renderings
        :return: rendered data
        """
        if mode in self.metadata['render.modes']:
            return self._display.get_camera_image(mode)
        else:
            # just raise an exception
            super(PerlsEnv, self).render(mode=mode)

    def _reset(self):
        """
        Reset the world environment.
        :return: Empty list of states. The state
        """
        self._world.reset()
        for _ in range(500):
            self._world.update()

    @abc.abstractmethod
    def _step(self, action):
        """
        Make one step move in the environment.
        :param action: Action as defined in action space
        :return: Observations, Rewards, isDone, Info tuple
        """
        return NotImplemented
