# !/usr/bin/env python

import abc

from .perls_env import PerlsEnv
from perls import math_util
import numpy as np 

# TODO: register gym env
# TODO: cutoff demons when cube z pos decreases??


class PushCube(PerlsEnv):
    """
    Pushing cube to a specific goal on table
    """
    def __init__(self, conf_path, max_step):

        super(PushCube, self).__init__(conf_path, max_step)
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
    def reward_range(self):
        """
        Get the min and max reward range as a tuple
        """
        return (0, 2)

    def _reset(self):
        """
        Override method.
        """
        # Set to top down view to align with real world
        # Overwrite settings in config file

        self._display.set_render_view(
            dict(
                dim=(150, 150),
                flen=2.,
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
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        goal_pos = self._world.get_task_state()['goal']

        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)

        cube_vel = self._cube.v

        return math_util.concat((np.cos(self._robot.joint_positions),   
                                 np.sin(self._robot.joint_positions),
                                 self._robot.joint_velocities / math_util.vec(self._robot.joint_specs['max_vel']),
                                 cube_pos, 
                                 #cube_orn,
                                 cube_vel,
                                 math_util.vec(goal_pos) - math_util.vec(cube_pos)))
        # State space: 7 + 7 + 7 + 3 + 3 + 3 = 30

        # return math_util.concat((self._robot.joint_positions,
        #                          self._robot.joint_velocities)),\
        #        math_util.concat((cube_pos, cube_orn, cube_vel,
        #                          # goal_pos, 
        #                          #math_util.vec(cube_pos) - math_util.vec(eef_pos),
        #                          math_util.vec(goal_pos) - math_util.vec(cube_pos)))

    def _step_helper(self, action):
        return NotImplemented
