# !/usr/bin/env python

from .push_cube import PushCube
from lib.utils import math_util


class PushViz(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path):

        super(PushViz, self).__init__(conf_path)

    @property
    def state(self):
        goal_pos = self._world.get_task_state()['goal']
        aux = math_util.concat((self._robot.joint_positions,
                               self._robot.joint_velocities,
                               goal_pos))
        return self._display.get_camera_image('rgbd'), aux
