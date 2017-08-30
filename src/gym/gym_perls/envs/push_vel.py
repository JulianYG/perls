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
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec(self._robot.joint_specs['max_vel']),
            high=math_util.vec(self._robot.joint_specs['max_vel'])
        )

    @property
    def state(self):
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        return math_util.concat(self._robot.joint_positions,
                                self._robot.joint_velocities,
                                cube_pos, cube_orn)

    def _step(self, action):

        # Use velocity control
        self._robot.joint_velocities = action

        # Step size 0.001 in bullet, align with 0.1 real world control
        for _ in range(100):
            self._world.update()
        return self.state, self.reward, self.done, {'state': self.state}
