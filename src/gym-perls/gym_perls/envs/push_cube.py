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
        self._table = self._world.body['table_0']

    @property
    def state(self):
        arm_state = self._robot.joint_positions + self._robot.joint_velocities
        cube_pose = self._cube.get_pose(self._robot.uid, 0)
        cube_state = list(cube_pose[0]) + list(cube_pose[1])
        return arm_state + cube_state

    @property
    def done(self):

        return False

    @property
    def reward(self):

        return self._robot.eef_pose - self._cube.get_pose(self._robot.uid, 0)

    def _reset(self):

        super(PushCube, self)._reset()

        # move robot to initial position
        # TODO: orientation offset
        # offset = self._robot.pinpoint(
        #     (0.65, 0.16, 0.24),
        #     # (-0.29, 0.189, 0.829),
        #     (0,1,0,0),
        #         # math_util.euler2quat([-math_util.pi, -math_util.pi / 2., 0.]),
        #     ftype='rel',max_iter=500)

        # self._robot.joint_states = (range(7), [0] * 7, 'position', None)

        return self.state

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action
        # TODO: make sure to go through IK here, since it's not perfect
        # TODO: then read robot state, and get the stuff we care about again. 

        self._robot.joint_torques = action
        self._world.update()

        return self.state, self.reward, self.done, {'state': self.state}
