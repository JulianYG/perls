
class InterruptHandler(object):

    def __init__(self, rate):

        self._rate = rate
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
    def signal(self):
        return self._signal


class NullHandler(InterruptHandler):
    """
    Singleton placeholder
    """
    def __init__(self, a=None, b=None):
        super(NullHandler, self).__init__(0.)
