# !/usr/bin/env python

import gym
import gym.spaces as spaces
import numpy as np
import logging
    
import sys, os
from perls.robot.sawyer import SawyerArm
import rospy

root_logger = logging.getLogger()
root_logger.setLevel(logging.WARNING)


class Reach(gym.Env):
    """
    Construct a gym environment for perls simulation
    """
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
        assert rospy.get_name() != '/unnamed', 'Must init node!'
        self._goal = np.array([.44, -.165, .13558])
        self._robot = SawyerArm(False)
        self._rate = rospy.Rate(5)
    
    @property
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.ones(7) * -.1,
            high=np.ones(7) * .1)

    @property
    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.array([-1] * 14 + [-.1] * 7),
            high=np.array([1] * 14 + [.1] * 7))

    @property
    def reward_range(self):
        """
        A tuple corresponding to the min and max possible rewards
        """
        return (0, 2)

    def _close(self):
        """
        Shutdown the environment
        :return: None
        """
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
        pass

    @property
    def _state(self):
        return np.concatenate((
            np.cos(self._robot.joint_positions), 
            np.sin(self._robot.joint_positions), 
            self._robot.joint_velocities))

    def _reset(self):
        """
        Reset the world environment.
        :return: The state defined by users constrained by
        state space.
        """
        print('********************* start *********************')
        # self._robot.set_timeout(0.15)

        # Initialize to random position
        rand_start = np.random.uniform((0.233, 0.347, 0), (0.545, 0.177, 0.521))
        # self._robot.tool_pose = (rand_start, (0, 1, 0, 0))
        self._robot.neutral()

        return self._state

    def _step(self, action):
        """
        Make one step move in the simulation,
        but aligned with real time elapsed.
        :param action: Action as defined in action space
        :return: Observations, Rewards, isDone, Info tuple
        """
        action = np.clip(action, -0.4, 0.4)
        
        self._robot.velocity_control_safe(action)
        self._rate.sleep()
        tool_pos = np.array(self._robot.tool_pose[0])
        done = True if np.allclose(self._goal, tool_pos, rtol=1e-2) else False
        rew = int(done) + np.exp(-1. * np.linalg.norm(self._goal - tool_pos, 2))

        return self._state, rew, done, \
            {'eef position': tool_pos, 'goal': self._goal}
