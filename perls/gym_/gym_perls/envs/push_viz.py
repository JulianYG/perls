# !/usr/bin/env python

from .push_cube import PushCube
from perls import math_util


class PushViz(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path, max_step):

        super(PushViz, self).__init__(conf_path, max_step)

    @property
    def observation_space(self):

        _, table_orn = self._table.pose

        goal_abs_lower = math_util.vec(
            (self._cube.pos[0] + 0.25, self._cube.pos[1] - 0.25, 0.641))
        goal_abs_upper = math_util.vec(
            (self._cube.pos[0] + 0.45, self._cube.pos[1] + 0.25, 0.642))

        goal_upper, _ = math_util.get_relative_pose(
            (goal_abs_upper, table_orn), self._robot.pose)
        goal_lower, _ = math_util.get_relative_pose(
            (goal_abs_lower, table_orn), self._robot.pose)

        img_space = \
            PushViz.Space.Box(
                low=math_util.zero_vec((96, 96, 4)),
                high=math_util.one_vec((96, 96, 4))
            )

        aux_space = \
            PushViz.Space.Box(
                low=math_util.concat((
                    math_util.vec(self._robot.joint_specs['lower']),
                    -math_util.vec(self._robot.joint_specs['max_vel']),
                    goal_lower,
                    (-1, -1, -1),
                    (-1, -1, -1))
                ),
                high=math_util.concat((
                    math_util.vec(self._robot.joint_specs['upper']),
                    math_util.vec(self._robot.joint_specs['max_vel']),
                    goal_upper,
                    (1, 1, 1),
                    (1, 1, 1))
                )
            )

        return PushViz.Space.Tuple((img_space, aux_space))

    @property
    def state(self):
        goal_pos = self._world.get_task_state()['goal']

        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)

        aux = math_util.concat((self._robot.joint_positions,
                               self._robot.joint_velocities,
                               goal_pos,
                               math_util.vec(cube_pos) - math_util.vec(eef_pos),
                               math_util.vec(goal_pos) - math_util.vec(cube_pos)))
        return self._display.get_camera_image('rgbd'), aux
