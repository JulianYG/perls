import abc

from ..utils.time_util import Timer


class ControlHandler(object):
    """
    Base class for control interrupt handling
    """
    def __init__(self, ps_id, queue, sensitivity, rate):
        # self._id = ps_id
        self._sens = sensitivity
        self._rate = rate
        self._handler = Timer(1. / rate, self.interrupt, None, queue)

    @property
    def freq(self):
        return 1. / self._rate

    @property
    def name(self):
        return 'ControlHandler'

    def run(self):
        self._handler.start()

    @abc.abstractmethod
    def interrupt(self, queue):
        return NotImplemented

    def pause(self):
        self._handler.pause()

    def resume(self):
        self._handler.resume()

    def stop(self):
        self._handler.cancel()


class NullHandler(ControlHandler):
    """
    Singleton placeholder
    """
    def __init__(self, ps_id, queue, sensitivity, rate):
        super(NullHandler, self).__init__(0, None, 0, rate)

    def interrupt(self, queue):
        return NotImplemented

    def stop(self):
        pass
