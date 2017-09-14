# !/usr/bin/env python

from .push_viz import PushViz
from lib.utils import math_util


class PushVizVel(PushViz):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path, max_step):

        super(PushVizVel, self).__init__(conf_path, max_step)

    @property
    def observation_space(self):
        return NotImplemented

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_vel']),
            high=math_util.vec(self._robot.joint_specs['max_vel'])
        )

    def _step_helper(self, action):
        # Use velocity control
        self._robot.joint_velocities = action
