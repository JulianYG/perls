# !/usr/bin/env python

import gym
import gym.spaces as spaces
import numpy as np
import logging

try:
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from utils.pcl_segment import PCLSegment
except ImportError:
    pass
    
import sys, os
from perls.robot.sawyer import SawyerArm


__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

root_logger = logging.getLogger()
root_logger.setLevel(logging.WARNING)


class PushRobot(gym.Env):
    """
    Construct a gym environment for perls simulation
    """
    Space = spaces
    metadata = {
        'render.modes': ['human', 'rgb', 'rgbd', 
                         'depth', 'segment'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        """
        Initialize the environment
        :param conf_path: the absolute path string to
        the configuration file, default is 'gym_-disp.xml'
        """
        rospy.init_node('robot_gym')
        self._goal = None
        self._pcl = PCLSegment()
        self._pcl.boot()
        self._robot = SawyerArm(False)

        self._cube_idx = 1
        
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return NotImplemented

    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return NotImplemented

    def reward_range(self):
        """
        A tuple corresponding to the min and max possible rewards
        """
        return NotImplemented

    def _close(self):
        """
        Shutdown the environment
        :return: None
        """
        cv2.destroyAllWindows()
        self._pcl.unregister()
        self._robot.shutdown()

    def _render(self, mode='rgb', close=False):
        """
        Generate rendered data based on given mode.
        :param mode: string of mode, as specified in metadata
        human: pop up a window and render
        rgb: an rgb array suitable for video
        depth: a depth value array of current env
        seg: segmentation value array
        :param close: close all open renderings
        :return: rendered data
        """
        img = CvBridge().imgmsg_to_cv2(self._pcl.rgb, 'bgr8')
        cube = self._pcl.objects.markers[self._cube_idx]

        cube_pos = np.array([cube.pose.position.x, cube.pose.position.y, cube.pose.position.z])
        cube_size = np.array([cube.scale.x, cube.scale.y, cube.scale.z]) / 2.

        u1, v1 = self._pcl.convert(*(cube_pos + cube_size) / 2.)
        u2, v2 = self._pcl.convert(*(cube_pos - cube_size) / 2.)
        cv2.rectangle(img, (u1, v1), (u2, v2), (0, 0, 255), 2)

        cv2.imshow('img', img)
        cv2.waitKey(0)

    def _reset(self):
        """
        Reset the world environment.
        :return: The state defined by users constrained by
        state space.
        """
        self._robot.set_timeout(0.15)
        self._pcl.update_obj()
        # self._pcl.update_table()

        cube = self._pcl.objects.markers[self._cube_idx]

        # Filter out other objects
        for idx, obj in enumerate(self._pcl.objects.markers):
            if PCLSegment.transform(obj.pose)[0][2] < -0.2:
                self._cube_idx = idx 
                cube = obj
                break

        table_pos = self._pcl.table.pose.position
        table_pos = np.array([table_pos.x, table_pos.y, table_pos.z])

        cube_pos, cube_orn = PCLSegment.transform(cube.pose)
        self._goal = cube_pos + np.random.uniform([-0.25, -0.05, 0], [0.25, -0.35, 0])

        # self._prev_pos = cube_pos
        print('Detected table position: {}, '
              'goal position: {}, cube position: {}'.format(
                table_pos, self._goal, cube_pos))

        return np.concatenate((
            self._robot.joint_positions, 
            self._robot.joint_velocities,
            cube_pos, cube_orn, self._goal))

    def _step(self, action):
        """
        Make one step move in the simulation,
        but aligned with real time elapsed.
        :param action: Action as defined in action space
        :return: Observations, Rewards, isDone, Info tuple
        """
        self._robot.joint_velocities = action
        self._pcl.update_obj()
        cube_pos, cube_orn = PCLSegment.transform(
            self._pcl.objects.markers[self._cube_idx].pose)

        done = True if np.allclose(self._goal, cube_pos, rtol=1e-2) else False

        # reward = np.sqrt(np.sum((self._prev_pos - self._goal) ** 2)) - \
        #     np.sqrt(np.sum((cube_pos - self._goal) ** 2))
        # self._prev_pos = cube_pos

        return np.concatenate((
            self._robot.joint_positions, 
            self._robot.joint_velocities,
            cube_pos, cube_orn, self._goal)), 0, done, \
            {'cube position': cube_pos, 'goal': self._goal}
