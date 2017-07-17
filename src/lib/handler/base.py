
class InterruptHandler(object):

    def __init__(self, ps_id, rate):

        self._rate = rate
        self._id = ps_id
        self._signal = dict(
            tid=0, key=None, cmd=list(), instruction=list())

    # TODO: Add remote some time in the future
    # @property
    # def remote(self):
    #     return
    #
    # @remote.setter
    # def remote(self, (ip, port)):
    #     pass
    #
    # @remote.deleter
    # def remote(self):
    #     pass

    @property
    def name(self):
        return 'InterruptHandler'

    @property
    def signal(self):
        return self._signal

    def stop(self):
        raise NotImplementedError('<stop> is not implemented for InterruptHandler')


class NullHandler(InterruptHandler):
    """
    Singleton placeholder
    """
    def __init__(self, a=None, b=None, c=None):
        super(NullHandler, self).__init__(0, 0.)
