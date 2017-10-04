# !/usr/bin/env python

from .push_cube import PushCube
from perls import math_util


class PushCubeVel(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path, max_step):

        super(PushCubeVel, self).__init__(conf_path, max_step)

    @property
    def observation_space(self):

        table_bound, table_orn = self._table.pose

        table_abs_upper_bound = table_bound + math_util.vec((.325, .2, 0.))
        table_abs_lower_bound = table_bound - math_util.vec((.325, .2, 0.))

        table_upper, _ = math_util.get_relative_pose(
            (table_abs_upper_bound, table_orn), self._robot.pose)
        table_lower, _ = math_util.get_relative_pose(
            (table_abs_lower_bound, table_orn), self._robot.pose)

        goal_abs_lower = math_util.vec(
            (self._cube.pos[0] + 0.25, self._cube.pos[1] - 0.25, 0.641))
        goal_abs_upper = math_util.vec(
            (self._cube.pos[0] + 0.45, self._cube.pos[1] + 0.25, 0.642))

        goal_upper, _ = math_util.get_relative_pose(
            (goal_abs_upper, table_orn), self._robot.pose)
        goal_lower, _ = math_util.get_relative_pose(
            (goal_abs_lower, table_orn), self._robot.pose)

        return PushCube.Space.Box(
            low=math_util.concat((
                math_util.vec(self._robot.joint_specs['lower']),
                -math_util.vec(self._robot.joint_specs['max_vel']),
                table_lower,
                (-1, -1, -1, -1),
                (0, 0, 0),
                #goal_lower,
                #(-1, -1, -1),
                (.25, -.25, -.1),
                )
            ),
            high=math_util.concat((
                math_util.vec(self._robot.joint_specs['upper']),
                math_util.vec(self._robot.joint_specs['max_vel']),
                table_upper,
                (1, 1, 1, 1),
                (1, 1, 1),
                (.45, .25, .1)
                )
                #goal_upper,
                #(1, 1, 1),
                #(1, 1, 1))
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
