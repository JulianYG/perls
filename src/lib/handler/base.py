import abc

from ..utils.time_util import Timer


class ControlHandler(object):
    """
    Base class for control interrupt handling
    """
    def __init__(self, ps_id, queue, sensitivity, rate, qsize):
        self._id = ps_id
        self._sens = sensitivity
        self._handler = Timer(1. / rate, self.interrupt, None, queue)

    @property
    def name(self):
        return 'ControlHandler'

    def run(self):
        self._handler.start()

    @abc.abstractmethod
    def interrupt(self, queue):
        return NotImplemented

    def stop(self):
        self._handler.cancel()


class NullHandler(ControlHandler):
    """
    Singleton placeholder
    """
    def __init__(self, a=None, b=None, c=None, d=None):
        super(NullHandler, self).__init__(0, None, 0, 1, 0)

    def update_states(self, state):
        return

    def stop(self):
        return
