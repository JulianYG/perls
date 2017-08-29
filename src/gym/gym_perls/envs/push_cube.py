# !/usr/bin/env python

from .perls_env import PerlsEnv
from lib.utils import math_util

# TODO: register gym env
# TODO: cutoff demons when cube z pos decreases??


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
    def action_space(self):
        return PerlsEnv.Space.Box(
            low=-self._robot.joint_specs['max_vel'],
            high=self._robot.joint_specs['max_vel']
        )

    @property
    def state(self):
        # arm_state = self._robot.joint_positions + self._robot.joint_velocities
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        cube_pos, cube_orn = self._cube.get_pose(self._robot.uid, 0)
        return math_util.concat(eef_pos, cube_pos, cube_orn)

    @property
    def done(self):

        # done if cube falls off table
        if self._cube.pos[2] < 0.6:
            return True
        return False

    @property
    def reward(self):

        # if cube goes out of bounds, give negative reward
        if (self._cube.pos[1] <= -0.58) or (self._cube.pos[1] >= -0.018):
            return -100

        # TODO: put negative reward for other side of table too?

        # square difference in x distance
        return - ((self._cube.pos[0] - 0.68) ** 2)

    def _reset(self):

        super(PushCube, self)._reset()
        self._display.set_render_view(
            dict(
                dim=(256, 256),
                flen=3,
                yaw=50,
                pitch=-35,
                focus=(0, 0, 0)
            )
        )

        cube_pos = self._cube.pos
        # Enable torque control by disable the motors first
        # As required by bullet
        # self._robot.torque_mode()

        self._robot.tool_pos = \
                ((cube_pos[0] - 0.05, cube_pos[1], cube_pos[2] + 0.025), 200)

        # move robot to initial position
        # TODO: orientation offset
        # offset = self._robot.pinpoint(
        #     (0.65, 0.16, 0.24),
        #     # (-0.29, 0.189, 0.829),
        #     (0,1,0,0),
        #         # math_util.euler2quat([-math_util.pi, -math_util.pi / 2., 0.]),
        #     ftype='rel',max_iter=500)

        return self.state

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action
        # TODO: make sure to go through IK here, since it's not perfect
        # TODO: then read robot state, and get the stuff we care about again. 

        # Use velocity control
        # self._robot.joint_velocities = action

        # Use end effector delta pose with iterations
        self._robot.tool_pos = (self._robot.tool_pos + math_util.vec(action), True)

        # rate / step size = 0.01 / 0.001 = 10 (account for 100 Hz sampling of demonstrations)
        self._world.update()
        return self.state, self.reward, self.done, {'state': self.state}
