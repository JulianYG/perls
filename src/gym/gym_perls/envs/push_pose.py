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
    def action_space(self):
        # No large movements
        return PushCube.Space.Box(
            low=-math_util.vec(0.01, 0.01, 0.01),
            high=math_util.vec(0.01, 0.01, 0.01)
        )

    @property
    def state(self):
        # arm_state = self._robot.joint_positions + self._robot.joint_velocities
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        return math_util.concat(eef_pos, cube_pos, cube_orn)

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action
        # TODO: make sure to go through IK here, since it's not perfect
        # TODO: then read robot state, and get the stuff we care about again. 

        # Use end effector delta pose with iterations
        self._robot.set_eef_pose(
            self._robot.eef_pose[0] + math_util.vec(action), 
            None, 100)

        return self.state, self.reward, self.done, {'state': self.state}

