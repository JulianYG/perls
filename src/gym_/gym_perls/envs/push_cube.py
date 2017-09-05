# !/usr/bin/env python

import abc

from .perls_env import PerlsEnv
from lib.utils import math_util

# TODO: register gym env
# TODO: cutoff demons when cube z pos decreases??


class PushCube(PerlsEnv):
    """
    Pushing cube to a specific goal on table
    """
    def __init__(self, conf_path):

        super(PushCube, self).__init__(conf_path)
        self._cube = self._world.body['cube_0']
        self._robot = self._world.tool['m0']
        self._table = self._world.body['table_0']

    @property
    def observation_space(self):
        """
        Get observation (state) ranges in the environment
        :return: Space object
        """
        return NotImplemented

    @property
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return NotImplemented

    def _reset(self):
        """
        Override method.
        """
        # Set to top down view to align with real world
        # Overwrite settings in config file
        self._display.set_render_view(
            dict(
                dim=(512, 424),
                flen=1.77,
                # Have to use exact numbers for aligned
                # Top down view in GUI and non-GUI modes...
                yaw=90.0001,
                pitch=-75,
                focus=self._world.body['table_0'].pos
            )
        )
        return super(PushCube, self)._reset()

    @property
    def state(self):
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        goal_pos = self._world.get_task_state()['goal']

        return math_util.concat(eef_pos, cube_pos, cube_orn, goal_pos)

    def _step_helper(self, action):
        return NotImplemented
