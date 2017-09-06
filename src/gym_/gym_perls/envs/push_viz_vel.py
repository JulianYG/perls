# !/usr/bin/env python

from .push_viz import PushViz
from lib.utils import math_util


class PushVizVel(PushViz):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path):

        super(PushVizVel, self).__init__(conf_path)

    @property
    def observation_space(self):
        return NotImplemented

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_vel']),
            high=math_util.vec(self._robot.joint_specs['max_vel'])
        )

    @property
    def state(self):

        img = super(PushVizVel, self).state
        goal_pos = self._world.get_task_state()['goal']
        aux = math_util.concat(self._robot.joint_positions,
                               self._robot.joint_velocities,
                               goal_pos)
        return img, aux

    def _step_helper(self, action):
        # Use velocity control
        self._robot.joint_velocities = action
