from .utils import math_util
from .utils.io_util import (FONT,
                            logerr,
                            loginfo)


class Adapter(object):
    """
    In this project only view2model adapter
    is needed, since model directly 
    writes into view
    """

    def __init__(self, model):
        """
        Initialize the adapter with given model
        :param model: the world
        """
        self._world = model

    #
    # def check_world_states(self):
    #     """
    #     Check the world states to see if the task
    #     is completed.
    #     :return: done, success
    #     """
    #     return False, 0

    def update_world(self, update_info):
        pass
    #
    # def set_world_states(self, name=('', '')):
    #     pass

