# !/usr/bin/env python

from ..utils import math_util
from ..utils.io_util import loginfo, FONT, pjoin, fdelete

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
        self._job = 'run'

        log_path = pjoin(
            __file__, 
            '../../../log/{}.txt'.format(env_name))

        # Appending to previous one if exists
        self._log_file = open(log_path, 'a')

    @property
    def name(self):
        return 'TaskCompletionChecker'

    @property
    def state(self):
        return self._states

    def set_job(self, job):
        self._job = job

    def initialize(self, world):

        """
        Customize world for some fine tunings that
        cannot be specified in env xml file
        :param world: the world object to be setup
        :return: None
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':

            # Fine tune the environment to look real
            world.body['plane_0'].set_texture(
                -1, 'floor', pjoin(__file__, '../../../../asset/floor.png'))

            cube = world.body['cube_0']

            cube.set_texture(
                -1, 'cube', pjoin(__file__, '../../../../asset/cube.png'))

            gripper = world.body['bax_0']

            gripper.color = (0, (0, 0, 0, 1))
            gripper.color = (1, (0, 0, 0, 1))
            gripper.color = (2, (0, 0, 0, 1))
            gripper.color = (3, (0, 0, 0, 1))
            gripper.color = (4, (0, 0, 0, 1))

            table = world.body['table_0']
            # table.set_texture(
            #     -1, 'table', pjoin(__file__, '../../../../asset/table.png'))

            # Random goal
            box_center = math_util.rand_vec(
                3, (cube.pos[0] + 0.25, cube.pos[1] - 0.25, 0.641),
                (cube.pos[0] + 0.45, cube.pos[1] + 0.25, 0.642),
                'uniform')

            self._states['goal'] = box_center
            self._states['goal_norm'] = math_util.l2(box_center - cube.pos)

            # Only add lines for GUI or demos
            if world.info['engine']['visual']:
                table.mark = ('box2d', 15, [1, 0, 0], None, 0,
                              {'center': box_center,
                               'size': 0.1,})

            cube_pos = cube.pos
            robot = world.body['titan_0']

            # Initializes the gripper next to the cube
            initial_gripper_pos = \
                (cube_pos[0] - 0.1, cube_pos[1], cube_pos[2] + 0.03)
            
            robot.tool_pos = (initial_gripper_pos, 300)

            # Use this as a mark
            robot.grasp(1)

            # self._states['cube_norm'] = math_util.l2(robot.tool_pos - cube.pos)
            self._states['last_delta'] = math_util.l2(self._states['goal'] - cube.pos)

            # loginfo('Initialize finished.', FONT.model)
            # loginfo('Initial joint positions: {}'.
            #         format(robot.joint_positions),
            #         FONT.model)
            # loginfo('Initial gripper finger position: {}'.
            #         format(robot.tool_pos),
            #         FONT.model)
            # loginfo('Initialized goal state: {}'.
            #         format(box_center),
            #         FONT.model)

    def score(self, world):
        """
        Score the current performance of the agent. Generates
        the reward / cost
        :param world: current environment status object
        :return: User defined format of reward
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':
            robot = world.body['titan_0']
            cube_pos = world.body['cube_0'].pos

            # dist_gripper = math_util.rms(robot.tool_pos - cube.pos)
            # dist_goal = math_util.rms(cube.pos - self._states['goal'])

            # Scale according to the env's initial states
            # dist_gripper_norm = math_util.l2((0.03,) * 3)

            curr_delta = math_util.l2(goal - cube_pos)
            reward = self._states['last_delta'] - curr_delta
            self._states['last_delta'] = math_util.l2(goal - cube_pos)

            # If the cube bumps or falls, penalize
            if cube_pos[2] >= 0.68 or cube_pos[2] <= 0.6:
                return -1

            # If robot/gripper collided with table, penalize
            for points in world.body['table_0'].contact:
                for point in points:
                    if point['uid_other'] < 2:
                        return -1

            tool_pos = world.body['titan_0'].tool_pos
            if math_util.l2(tool_pos - cube_pos) > 0.3:
                return -1

            # If cube is within the boundary, award
            
            goal = self._states['goal']
            if goal[0] - .05 < cube_pos[0] < goal[0] + .05 \
               and goal[1] - .05 < cube_pos[1] < goal[1] + .05:
                return reward + 1

            # return 1. / (dist_gripper * .7 / self._states['cube_norm']
            #           + dist_goal * .3 / self._states['goal_norm']) - penalty
            # print(- dist_goal / self._states['goal_norm'] - penalty)
            # return - dist_goal / self._states['goal_norm'] - penalty

            return reward

    def check(self, world):

        if self._name == 'push_sawyer' or self._name == 'push_kuka':

            cube_pos = world.body['cube_0'].pos

            # If the cube falls, fail directly
            if cube_pos[2] <= 0.6:
                return True, False

            # If collided with table, fail
            for points in world.body['table_0'].contact:
                for point in points:
                    if point['uid_other'] < 2:
                        return True, False

            # If gripper too far away from the cube, fail
            tool_pos = world.body['titan_0'].tool_pos
            if math_util.l2(tool_pos - cube_pos) > 0.3:
                return True, False

            # If cube is within the boundary, success
            goal = self._states['goal']
            if goal[0] - .05 < cube_pos[0] < goal[0] + .05 \
               and goal[1] - .05 < cube_pos[1] < goal[1] + .05:
                if self._job == 'record':
                    # In success case, take down the goal pos
                    self._log_file.write('{}\n'.format(
                        ' '.join(str(x) for x in goal)))
                return True, True

        return False, False

    def stop(self):
        self._log_file.close()
