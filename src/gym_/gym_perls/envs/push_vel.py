# !/usr/bin/env python

from .push_cube import PushCube
from lib.utils import math_util


class PushCubeVel(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path):

        super(PushCubeVel, self).__init__(conf_path)

    @property
    def observation_space(self):

        table_bound, table_orn = self._table.pose

        table_abs_upper_bound = table_bound + math_util.vec((.325, .2, 0.))
        table_abs_lower_bound = table_bound - math_util.vec((.325, .2, 0.))

        table_upper = math_util.get_relative_pose(
            (table_abs_upper_bound, table_orn), self._robot.pose)
        table_lower = math_util.get_relative_pose(
            (table_abs_lower_bound, table_orn), self._robot.pose)

        goal_pos = self._world.get_task_state()['goal']

        return PushCube.Space.Box(
            low=math_util.concat((
                math_util.vec(self._robot.joint_specs['lower']),
                -math_util.vec(self._robot.joint_specs['max_vel']),
                table_lower,
                (-1, -1, -1, -1),
                goal_pos)
            ),
            high=math_util.concat((
                math_util.vec(self._robot.joint_specs['upper']),
                math_util.vec(self._robot.joint_specs['max_vel']),
                table_upper,
                (1, 1, 1, 1),
                goal_pos)
            )
        )

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_vel']),
            high=math_util.vec(self._robot.joint_specs['max_vel'])
        )

    def _step_helper(self, action):
        # Use velocity control
        self._robot.joint_velocities = action
