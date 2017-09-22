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
