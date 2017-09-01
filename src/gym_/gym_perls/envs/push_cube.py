# !/usr/bin/env python

import abc

from .perls_env import PerlsEnv
from lib.utils import math_util

# TODO: register gym env
# TODO: cutoff demons when cube z pos decreases??


class PushCube(PerlsEnv):
    """
    Pushing cube across the table
    """

    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

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

    @property
    def state(self):
        # arm_state = self._robot.joint_positions + self._robot.joint_velocities
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        #goal_pos = self._world.get_task_state()['goal']
        #return math_util.concat(eef_pos, cube_pos, cube_orn, goal_pos)
        return math_util.concat(eef_pos, cube_pos, cube_orn)

    def _step_helper(self, action):
        return NotImplemented
