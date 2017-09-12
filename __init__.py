
import logging
import platform

from .src.gym_ import gym_perls
from .src.lib.entity import body

from .src.lib.utils import (math_util,
                           io_util,
                           plot_util,
                           time_util,
                           postprocess,
                           event_listener)

from .src.lib.control import Controller

from .src.pyrobots import kinect, sawyer

logging.setLoggerClass(io_util.PerlsLogger)
