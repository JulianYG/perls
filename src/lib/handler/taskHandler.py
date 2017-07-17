from .base import InterruptHandler

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class Checker(InterruptHandler):
    """
    Check if given task is finished
    """
    def __init__(self, ps_id, env_name):
        super(Checker, self).__init__(ps_id, 100)
        self._name = env_name

    def check(self):
        if self._name == 'scoop':

            pass

        return False, False

