from .perls_env import PerlsEnv


class PushCube(PerlsEnv):

    """
    Trying to solve InverseKinematics by RL
    """
    metadata = {
        'render.modes': ['human', 'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self, conf_path):

        super(PushCube, self).__init__(conf_path)

    def _reset(self):

        super(PushCube, self)._reset()
        return self._get_relative_pose()

    def _get_relative_pose(self):

        cube = self._world.body['cube_0']
        tool = self._world.tool['m0']
        cube_pose_rel = cube.get_pose(tool.uid, 0)
        eef_pose_rel = tool.tool_pose_rel

        return eef_pose_rel, cube_pose_rel

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action

        # TODO: make sure to go through IK here, since it's not perfect

        # TODO: then read robot state, and get the stuff we care about again. 

        return NotImplemented
