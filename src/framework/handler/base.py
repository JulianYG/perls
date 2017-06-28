
class InterruptHandler(object):

    def __init__(self, sensitivity, rate):

        self._rate = rate
        self._sens = sensitivity
        self._signal = dict(tid=0, key=None,
                            cmd=list(), instruction=list())

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

    def _parse_interrupt(self):
        raise NotImplementedError('<_parse_interrupt> is not '
                                  'implemented for this class')


class NullHandler(InterruptHandler):
    """
    Singleton placeholder
    """
    def __init__(self):
        InterruptHandler.__init__(self, 1., 0.)

    def _parse_interrupt(self):
        return 0

