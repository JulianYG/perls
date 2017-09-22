# !/usr/bin/env python

from ..utils import math_util
from ..utils.io_util import pjoin, fdelete, PerlsLogger

import logging 

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

logging.setLoggerClass(PerlsLogger)


class Checker(object):
    """
    Check if given task is finished
    """
    def __init__(self, env_name):
        """
        Initialize task checker.
        :param env_name: string of environment
        description name, as defined in world.xml
        scene tag.
        """
        self._name = env_name
        self._states = dict()
        self._job = 'run'

        log_path = pjoin(
            __file__, 
            '../../log/{}.txt'.format(env_name))

        # Appending to previous one if exists
        self._log_file = open(log_path, 'a')

    @property
    def name(self):
        """
        Get name of this checker.
        :return: name string of task
        """
        return 'Checker::{}'.format(self._name)

    @property
    def state(self):
        """
        Get task checker states
        :return: dictionary where key is state name
        and value is state value
        """
        return self._states

    def set_job(self, job):
        """
        Tell task checker what's the current run's job
        :param job: job string among 'record',
        'run', and 'replay'
        :return: None
        """
        self._job = job

    def initialize(self, world):
        """
        Customize world for some fine tunings that
        cannot be specified in env xml file
        :param world: the world object to be setup
        :return: None
        """
        if self._name == 'push_sawyer':

            # Fine tune the environment to look real
            world.body['plane_0'].set_texture(
                -1, 'floor', pjoin(__file__, '../../asset/floor.png'))

            cube = world.body['cube_0']

            cube.set_texture(
                -1, 'cube', pjoin(__file__, '../../asset/cube.png'))

            gripper = world.body['bax_0']

            gripper.color = (0, (0, 0, 0, 1))
            gripper.color = (1, (0, 0, 0, 1))
            gripper.color = (2, (0, 0, 0, 1))
            gripper.color = (3, (0, 0, 0, 1))
            gripper.color = (4, (0, 0, 0, 1))

            table = world.body['table_0']
            table.dynamics = {-1: dict(lateral_friction=1.0)}
            # table.set_texture(
            #     -1, 'table', pjoin(__file__, '../../asset/table.png'))

            # Random goal
            box_center = math_util.rand_vec(
                3, (cube.pos[0] + 0.25, cube.pos[1] - 0.25, 0.641),
                (cube.pos[0] + 0.45, cube.pos[1] + 0.25, 0.642),
                'uniform')

            robot_pose = world.body['titan_0'].pose
            goal_pos, _ = math_util.get_relative_pose(
                (box_center, robot_pose[1]), robot_pose)

            self._states['goal'] = goal_pos
            self._states['goal_abs'] = box_center

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

            self._states['last_delta'] = math_util.l2(box_center - cube.pos)

            logging.info('Initialize finished.')
            logging.info('Initial joint positions: {}'.
                    format(robot.joint_positions))
            logging.info('Initial gripper finger position: {}'.
                    format(robot.tool_pos))
            logging.info('Initialized goal state: {}'.
                    format(box_center))

    def score(self, world):
        """
        Score the current performance of the agent. Generates
        the reward / cost
        :param world: current environment status object
        :return: User defined format of reward
        """
        if self._name == 'push_sawyer':
            robot = world.body['titan_0']
            cube_pos = world.body['cube_0'].pos
            goal = self._states['goal_abs']

            # dist_gripper = math_util.rms(robot.tool_pos - cube.pos)
            # dist_goal = math_util.rms(cube.pos - self._states['goal'])

            # Scale according to the env's initial states
            # dist_gripper_norm = math_util.l2((0.03,) * 3)

            curr_delta = math_util.l2(goal - cube_pos)

            # reward = self._states['last_delta'] - curr_delta
            # self._states['last_delta'] = math_util.l2(goal - cube_pos)

            reward = - math_util.l2(goal, cube_pos) - \
                math_util.l2(robot.eef_pose[0], cube_pos)

            v_obj_eef = math_util.l2(cube_pos - robot.eef_pose[0])
            v_goal_obj = math_util.l2(goal - cube_pos)

            reward += v_obj_eef.dot(v_goal_obj)

            # If the cube bumps or falls, penalize
            if cube_pos[2] >= 0.69 or cube_pos[2] <= 0.6:
                return -10

            # If robot/gripper collided with table, penalize
            for points in world.body['table_0'].contact:
                for point in points:
                    if point['uid_other'] < 2:
                        return -10

            tool_pos = world.body['titan_0'].tool_pos
            if math_util.l2(tool_pos - cube_pos) > 0.3:
                return -10

            # If cube is within the boundary, award
            # if goal[0] - .05 < cube_pos[0] < goal[0] + .05 \
            #    and goal[1] - .05 < cube_pos[1] < goal[1] + .05:
            #     return reward + 10

            return reward

    def check(self, world):
        """
        Perform task completion check
        :param world: the current world task running in
        :return: Boolean done, boolean success
        """
        if self._name == 'push_sawyer':

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
            goal = self._states['goal_abs']
            if goal[0] - .05 < cube_pos[0] < goal[0] + .05 \
               and goal[1] - .05 < cube_pos[1] < goal[1] + .05:
                if self._job == 'record':

                    # In success case, take down the goal pos
                    # as bookkeeping for post-processing
                    self._log_file.write('{}\n'.format(
                        ' '.join(str(x) for x in self._states['goal'])))
                return True, True

        return False, False

    def stop(self):
        """
        Stop Task checker
        :return: None
        """
        self._log_file.close()
