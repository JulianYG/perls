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
        tool_pose, body_pose = self._world.get_states(
            ('tool', 'pose'), ('body', 'pose'))

        return tool_pose['m0'], body_pose['cube_0']

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action

        # TODO: make sure to go through IK here, since it's not perfect

        # TODO: then read robot state, and get the stuff we care about again. 

        return NotImplemented
