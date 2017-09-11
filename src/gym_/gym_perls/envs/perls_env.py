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
    Construct a gym environment for perls simulation
    """
    Space = spaces
    metadata = {
        'render.modes': ['human', 'rgb', 'rgbd', 
                         'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):
        """
        Initialize the environment
        :param conf_path: the absolute path string to
        the configuration file, default is 'gym_-disp.xml'
        """
        conf = io_util.parse_config(conf_path)[0]
        self._align_iters = 1

        _, self._world, self._display, control = Controller.load_config(conf, None)

        self._world.boot(self._display.info['frame'])
        self._status = self._display.run(None)

        if not self._world.info['engine']['real_time']:
            step_size = self._world.info['engine']['step_size']
            control_rate = control.freq
            self._align_iters = int(control_rate / step_size)

        # Store last action for regularization purposes
        self._action = None

        # Override Env class to prevent opening window after close
        self._owns_render = False

    @abc.abstractproperty
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return NotImplemented

    @abc.abstractproperty
    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return NotImplemented

    @abc.abstractproperty
    def reward_range(self):
        """
        A tuple corresponding to the min and max possible rewards
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
        done, _ = self._world.check_states()
        return done

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
        :return: The state defined by users constrained by
        state space.
        """
        self._world.reset()
        self._display.show()
        return self.state

    def _step(self, action):
        """
        Make one step move in the simulation,
        but aligned with real time elapsed.
        :param action: Action as defined in action space
        :return: Observations, Rewards, isDone, Info tuple
        """
        self._action = action
        # Perform extra steps in simulation to align
        # with real time
        for _ in range(self._align_iters):
            self._step_helper(action)
            self._world.update()

        return self.state, self.reward, self.done, {'state': self.state}

    @abc.abstractmethod
    def _step_helper(self, action):
        """
        The actual stepping function that needs to be implemented
        by children classes. This is necessary so that base class
        can align the simulation time with real gym_ control time.
        :param action: actions to be executed during step
        :return: None
        """
        return NotImplemented
