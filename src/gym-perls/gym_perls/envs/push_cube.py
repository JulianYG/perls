# !/usr/bin/env python

from .perls_env import PerlsEnv
from lib.utils import math_util


class PushCube(PerlsEnv):
    """
    Pushing cube across the table
    """

    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):

        super(PushCube, self).__init__(conf_path)
        self._cube = self._world.body['cube_0']
        self._robot = self._world.tool['m0']

    def _reset(self):

        super(PushCube, self)._reset()

        # move robot to initial position
        # TODO: orientation offset
        offset = self._robot.pinpoint(
            (0.65, 0.16, 0.24),
            # (-0.29, 0.189, 0.829),
            (0,1,0,0),
                # math_util.euler2quat([-math_util.pi, -math_util.pi / 2., 0.]),
            ftype='rel',max_iter=500)
        print(offset)
        # self._robot.joint_states = (range(7), [0] * 7, 'position', None)

        return self._get_relative_pose()

    def _get_relative_pose(self):

        cube_pose_rel = self._cube.get_pose(self._robot.uid, 0)
        eef_pose_rel = self._robot.tool_pose_rel

        return eef_pose_rel, cube_pose_rel

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action

        # TODO: make sure to go through IK here, since it's not perfect

        # TODO: then read robot state, and get the stuff we care about again. 

        self._world.update()
        # print(math_util.get_relative_pose((self._robot.kinematics['abs_frame_pos'][6], self._robot.kinematics['abs_frame_orn'][6]), (self._robot.pose)))