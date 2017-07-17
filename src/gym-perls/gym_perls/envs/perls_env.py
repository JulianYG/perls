import gym
from gym.utils import seeding

import sys, os
import abc
sys.path.append(os.path.abspath(os.path.join(__file__, '../../../../')))
from lib.control import Controller
from lib.utils import io_util


class PerlsEnv(gym.Env):
    """
    Construct an gym environment
    """

    metadata = {
        'render.modes': ['human', 'rgb', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):
        """
        Initialize the environment
        :param conf_path: the absolute path string to
        the configuration file, default is 'gym.xml'
        """
        conf = io_util.parse_config(conf_path)[0]
        self._world, self._display, _ = Controller.load_config(conf)
        self._status = self._display.run(None)
        self._world.boot(self._display.info['frame'])

    def _seed(self, seed=None):
        """
        Set the random seed for environment
        :param seed: integer seed number
        :return: generated seeds
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _close(self):
        """
        Shutdown the environment
        :return: None
        """
        self._world.clean_up()
        self._display.close()

    def _render(self, mode='', close=False):
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
        return

    @abc.abstractmethod
    def _step(self, action):
        """
        Make one step move in the environment.
        :param action: Action as defined in action space
        :return: Observations, Rewards, IfDone, Info tuple
        """
        return NotImplemented
