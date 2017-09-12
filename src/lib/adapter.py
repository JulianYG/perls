from .utils import math_util


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

    def update_world(self, update_info):
        pass

    # def set_world_states(self, name=('', '')):
    #     pass

