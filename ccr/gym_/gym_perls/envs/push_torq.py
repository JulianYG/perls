# !/usr/bin/env python

from .push_cube import PushCube
from perls import math_util


class PushCubeTorque(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path, max_step):

        super(PushCubeTorque, self).__init__(conf_path, max_step)

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
                goal_lower,
                (-1, -1, -1),
                (-1, -1, -1))
            ),
            high=math_util.concat((
                math_util.vec(self._robot.joint_specs['upper']),
                math_util.vec(self._robot.joint_specs['max_vel']),
                table_upper,
                (1, 1, 1, 1),
                goal_upper,
                (1, 1, 1),
                (1, 1, 1))
            )
        )

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_force']),
            high=math_util.vec(self._robot.joint_specs['max_force'])
        )

    @property
    def state(self):
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        goal_pos = self._world.get_task_state()['goal']

        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        
        return math_util.concat((
            self._robot.joint_positions,
            self._robot.joint_velocities,
            cube_pos, cube_orn, goal_pos,
            math_util.vec(cube_pos) - math_util.vec(eef_pos),
            math_util.vec(goal_pos) - math_util.vec(cube_pos)))

    def _reset(self):

        super(PushCubeTorque, self)._reset()
        self._robot.torque_mode()

    def _step_helper(self, action):

        # Use torque control
        self._robot.joint_torques = action
