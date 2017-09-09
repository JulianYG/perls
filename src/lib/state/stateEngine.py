#!/usr/bin/env python

import abc

from .base import StateEngine
from ..utils.io_util import logerr, FONT

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class RealStateEngine(StateEngine):
    """
    The robot state render (real)
    """
    def __init__(self, e_id, max_run_time, *_):
        """
        Constructor of robot state render.
        :param e_id: render id.
        :param max_run_time: maximum run time of render (robot)
        :param _: TBD
        """
        super(RealStateEngine, self).__init__(e_id, max_run_time)

        # The render needs to actually store a robot here?

    @abc.abstractmethod
    def configure(self, configs):
        """
        Configure the real state render (robot)
        :param configs: configuration
        :return: None
        """
        return NotImplemented


class FakeStateEngine(StateEngine):
    """
    The simulation state render (fake)
    """
    def __init__(self, e_id, max_run_time, async, step_size):
        """
        Constructor of simulation state render.
        :param e_id: render id label
        :param max_run_time: maximum run time of render (simulation)
        :param async: boolean if asynchronous simulation
        :param step_size: float of time step length, if async
        """
        super(FakeStateEngine, self).__init__(e_id, max_run_time)

        # real_time flag indicates current state (e.g., during
        # loading or before set real time simulation), while
        # async flag is constant
        self._real_time = False
        self._async = async

        if async:
            self._step_size = step_size
            if not isinstance(max_run_time, int):
                logerr('Need to specify integer max time steps '
                       'for asynchronous simulation!', FONT.control)
                return
            self._max_run_time = int(max_run_time)
            self._step_count = 0
        else:
            self._step_size = 0.0041666
            self._max_run_time = max_run_time

    @abc.abstractmethod
    def configure_environment(self, gravity, *_):
        """
        Configure the simulation environment, currently only
        set the gravity of the world
        :param gravity: float scalar to scale z-axis gravity,
        default is 1.0
        :param _: TBD
        :return: None
        """
        return NotImplemented
