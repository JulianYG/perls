# !/usr/bin/env python

from .push_cube import PushCube
from lib.utils import math_util


class PushCubeVel(PushCube):
    """
    Pushing cube across the table
    """

    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):

        super(PushCubeVel, self).__init__(conf_path)

    @property
    def observation_space(self):
        return PushCube.Space.box(
            low=math_util.concat(
                math_util.vec(self._robot.joint_specs['lower']),
                -math_util.vec(self._robot.joint_specs['max_vel']),
                self._table.pos - math_util.vec((.275, .275, -.63)),
                (-1, -1, -1, -1)
            ),
            high=math_util.concat(
                math_util.vec(self._robot.joint_specs['upper']),
                math_util.vec(self._robot.joint_specs['max_vel']),
                self._table.pos + math_util.vec((.275, .275, .69)),
                (1, 1, 1, 1)
            )
        )

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_vel']),
            high=math_util.vec(self._robot.joint_specs['max_vel'])
        )

    @property
    def state(self):
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        # goal_pos = self._world.get_task_state()['goal']
        
        # return math_util.concat(self._robot.joint_positions,
        #                         self._robot.joint_velocities,
        #                         cube_pos, cube_orn,
        #                         goal_pos)
        print(self._table.pos)
        return math_util.concat(self._robot.joint_positions,
                                self._robot.joint_velocities,
                                cube_pos, cube_orn)

    def _step_helper(self, action):
        # Use velocity control
        self._robot.joint_velocities = action
