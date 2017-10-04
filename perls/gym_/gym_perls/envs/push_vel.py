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

        # robot_space = PushCube.Space.Box(
        #     low=math_util.concat((
        #         math_util.vec(self._robot.joint_specs['lower']),
        #         -math_util.vec(self._robot.joint_specs['max_vel']))),
        #     high=math_util.concat((
        #         math_util.vec(self._robot.joint_specs['upper']),
        #         math_util.vec(self._robot.joint_specs['max_vel']))))



        # cube_space = PushCube.Space.Box(
        #     low=math_util.concat((
        #         table_lower,    # pos
        #         (-1, -1, -1, -1),   # orn
        #         (-1, -1, -1)    # vel
        #         (.25, -.25, -.1)
        #         )),
        #     high=math_util.concat((
        #         table_upper,
        #         (1, 1, 1, 1),
        #         (1, 1, 1),
        #         (.45, .25, .1)))
        # )

        # return PushViz.Space.Tuple((robot_space, cube_space))

        return PushViz.Space.Box(
            low=math_util.concat((
                math_util.vec(self._robot.joint_specs['lower']),
                -math_util.vec(self._robot.joint_specs['max_vel']),
                table_lower,    # pos
                (-1, -1, -1, -1),   # orn
                (-1, -1, -1),    # vel
                (.25, -.25, -.1)
            )),
            high=math_util.concat((
                math_util.vec(self._robot.joint_specs['lower']),
                math_util.vec(self._robot.joint_specs['max_vel']),
                table_upper,
                (1, 1, 1, 1),
                (1, 1, 1),
                (.45, .25, .1)
            ))
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
