from . import PerlsEnv
from lib.control import Controller
from lib.utils import io_util


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

    def _close(self):
        return NotImplemented

    def _render(self, mode='', close=False):
        return NotImplemented

    def _reset(self):
        return self._world.get_states()

    def _step(self, action):
        return NotImplemented
