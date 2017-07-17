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
        # tool_pose = self._world.get_states(('tool', 'pose'))
        tool_jpos = self._world.get_states(('tool', 'joint_states'))
        body_pose = self._world.get_states(('body', 'pose'))

        return tool_jpos['titan'], body_pose['cube_0']

    def _render(self, mode='', close=False):
        return NotImplemented

    def _step(self, action):
        return NotImplemented
