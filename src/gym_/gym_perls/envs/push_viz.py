# !/usr/bin/env python

from .push_cube import PushCube
from lib.utils import math_util


class PushViz(PushCube):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path):

        super(PushViz, self).__init__(conf_path)

    @property
    def state(self):
        
        return self._display.get_camera_image('rgb')
