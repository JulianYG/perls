import abc


class InterruptHandler(object):

    def __init__(self, ps_id, rate):

        self._rate = rate
        self._id = ps_id
        self._signal = dict(
            tid=0, key=None, cmd=list(), instruction=list())

    @property
    def name(self):
        return 'InterruptHandler'

    @property
    def signal(self):
        return self._signal

    @abc.abstractmethod
    def stop(self):
        raise NotImplementedError('<stop> is not implemented for InterruptHandler')


class NullHandler(InterruptHandler):
    """
    Singleton placeholder
    """
    def __init__(self, a=None, b=None, c=None):
        super(NullHandler, self).__init__(0, 0.)

    def update_states(self, state):
        return

    def stop(self):
        return
