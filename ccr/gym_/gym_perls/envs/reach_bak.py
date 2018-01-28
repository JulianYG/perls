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

    def __init__(self, hz=200, goal=[.44, -.17, .14], action_space_scale=0.2):
        """
        Initialize the environment
        :param conf_path: the absolute path string to
        the configuration file, default is 'gym_-disp.xml'
        """
        assert(action_space_scale > 0)
        assert rospy.get_name() != '/unnamed', 'Must init node!'
        self._goal = np.array(goal)
        self._robot = SawyerArm()
        self._robot.spin(hz, ctype='velocity')
        self._scale = action_space_scale
    
    @property
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.ones(7) * -self._scale,
            high=np.ones(7) * self._scale)

    @property
    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.array([-1] * 14 + [-self._scale] * 7),
            high=np.array([1] * 14 + [self._scale] * 7))

    @property
    def reward_range(self):
        """
        A tuple corresponding to the min and max possible rewards
        """
        return (-1, 1)

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
        self._robot.reset()
        return self._state

    def _step(self, action):
        """
        Make one step move in the simulation,
        but aligned with real time elapsed.
        :param action: Action as defined in action space
        :return: Observations, Rewards, isDone, Info tuple
        """
        # Scaling action
        action *= self._scale
        # print(action)
        self._robot.cmd = action
        
        tool_pos = np.array(self._robot.tool_pose[0])
        done = True if np.allclose(self._goal, tool_pos, rtol=1e-2) else False
        rew = int(done) + np.exp(-10. * np.linalg.norm(self._goal - tool_pos, 2)) - 1
        return self._state, rew, done, \
            {'eef position': tool_pos, 'goal': self._goal}
