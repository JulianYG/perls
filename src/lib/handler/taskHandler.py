# !/usr/bin/env python

from ..utils import math_util
from ..utils.io_util import loginfo, FONT

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

    @property
    def state(self):
        return self._states

    def initialize(self, world):

        """
        Customize world for some fine tunings that
        cannot be specified in env xml file
        :param world: the world object to be setup
        :return: None
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':

            table = world.body['table_0']

            # Random goal
            box_center = math_util.rand_vec(
                3, (table.pos[0] + 0.1, table.pos[1] - 0.25, 0.641),
                (table.pos[0] + 0.25, table.pos[1] + 0.25, 0.642),
                'uniform')

            # Fixed goal
            # box_center = math_util.vec((0.6, -0.2, 0.641))
        
            self._states['goal'] = box_center

            # Only add lines for GUI or demos
            if world.info['engine']['visual']:
                table.mark = ('box2d', 15, [1, 0, 0], None, 0,
                              {'center': box_center,
                               'size': 0.1,})

            cube_pos = world.body['cube_0'].pos
            robot = world.body['titan_0']

            # Initializes the gripper next to the cube
            initial_gripper_pos = \
                (cube_pos[0] - 0.05, cube_pos[1], cube_pos[2] + 0.025)
            
            robot.tool_pos = (initial_gripper_pos, 300)

            # Use this as a mark
            robot.grasp(1)

            loginfo('Initialize finished.', FONT.model)
            loginfo('Initial joint positions: {}'.
                    format(robot.joint_positions),
                    FONT.model)
            loginfo('Initial gripper finger position: {}'.
                    format(robot.tool_pos),
                    FONT.model)
            loginfo('Initialized goal state: {}'.
                    format(box_center),
                    FONT.model)

    def score(self, world):
        """
        Score the current performance of the agent. Generates
        the reward
        :param world: current environment status object
        :return: User defined format of reward
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':
            robot = world.body['titan_0']
            goal = self._states['goal']
            cube = world.body['cube_0']

            cost_grip = math_util.pos_diff(robot.tool_pos, cube.pos)
            cost_goal = math_util.pos_diff(cube.pos, self._states['goal'])

            return -(cost_grip * .8 + cost_goal * .2)

    def check(self, world):

        body_dict = world.body

        if self._name == 'push_sawyer' or self._name == 'push_kuka':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            # If cost too high, mark fail and done
            if -self.score(world) > .2:
                return True, False

            # If collided with table, fail
            for points in body_dict['bax_0'].contact:
                if points:
                    if points[0]['uid_other'] == 3:
                        return True, False

            if cube.pos[2] >= 0.68 or cube.pos[2] <= 0.6:
                # If the cube bumps or falls
                return True, False

            # Check if cube is within the boundary
            goal = self._states['goal']

            cube_pos = cube.pos
            if goal[0] - .05 < cube_pos[0] < goal[0] + .05 \
               and goal[1] - .05 < cube_pos[1] < goal[1] + .05:
                return True, True

        return False, False

    def stop(self):
        return
