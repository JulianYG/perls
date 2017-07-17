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
        """
        Reset the world environment
        :return: All initial states of the environment
        """
        all_states = super(PushCube, self)._reset()

        # TODO: get Robot end effector 2D pose in robot frame, get Cube 2D pose in robot frame

        return all_states

    def _render(self, mode='', close=False):
        return NotImplemented

    def _step(self, action):

        # TODO: action should be delta Robot end effector 2D pose, so do bounds clipping and apply action

        # TODO: make sure to go through IK here, since it's not perfect

        # TODO: then read robot state, and get the stuff we care about again. 

        return NotImplemented
