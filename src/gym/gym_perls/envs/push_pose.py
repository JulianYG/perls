# !/usr/bin/env python

from .push_cube import PushCube
from lib.utils import math_util


class PushCubePose(PushCube):
    """
    Pushing cube across the table
    """

    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):

        super(PushCubePose, self).__init__(conf_path)

    @property
    def observation_space(self):
        return PushCube.Space.box(
            low=math_util.concat(
                self._robot.pos - math_util.vec((1.5, 1.5, 1.5)),
                self._table.pos - math_util.vec((.275, .275, -.63)),
                (-1, -1, -1, -1)
            ),
            high=math_util.concat(
                self._robot.pos + math_util.vec((1.5, 1.5, 1.5)),
                self._table.pos + math_util.vec((.275, .275, .69)),
                (1, 1, 1, 1)
            )
        )

    @property
    def action_space(self):

        # No large movements
        return PushCube.Space.Box(
            low=-math_util.vec((0.01, 0.01, 0.01)),
            high=math_util.vec((0.01, 0.01, 0.01))
        )

    def _step_helper(self, action):

        # Use end effector delta pose with iterations
        self._robot.set_eef_pose(
            self._robot.eef_pose[0] + math_util.vec(action),
            None, iters=-1
        )
