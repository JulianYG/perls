# !/usr/bin/env python

from ..utils import math_util

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class Checker(object):
    """
    Check if given task is finished
    """
    def __init__(self, env_name):
        self._name = env_name

        self._states = dict()

    @property
    def name(self):
        return 'TaskCompletionChecker'

    def initialize(self, world):

        """
        Customize world for some fine tunings that
        cannot be specified in env xml file
        :param world: the world object to be setup
        :return: None
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':

            table = world.body['table_0']
            # table.mark = {}

            cube_pos = world.body['cube_0'].pos
            robot = world.body['titan_0']

            # Initializes the gripper next to the cube
            initial_gripper_pos = \
                (cube_pos[0] - 0.05, cube_pos[1], cube_pos[2] + 0.025)

            robot.tool_pos = (initial_gripper_pos, 200)

    def score(self, world):
        """
        Score the current performance of the agent. Generates
        the reward
        :param world: current environment status object
        :return: User defined format of reward
        """
        # TODO
        pass

    def check(self, body_dict):

        if self._name == 'push_sawyer':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            if cube.pos[2] >= 0.69 or cube.pos[2] <= 0.6:
                # If the cube bumps or falls
                return True, False

            # Check if cube is within the boundary
            if 1:

                return True, True

        elif self._name == 'push_kuka':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            if cube.pos[2] >= 0.69:
                # If the cube jumps too high
                return True, False
                
            # Consider the case when the cube is down on the ground
            if cube.pos[2] <= 0.06:
                done = True
                success = False

                # Only allow pushing towards one side, 
                # falling into one specific region
                if table.pos[1] - .275 <= cube.pos[1] <= table.pos[1] + .275 and \
                   table.pos[0] + .25 <= cube.pos[0] <= table.pos[0] + .65:
                    success = True

                return done, success

        return False, False

    def stop(self):
        return
