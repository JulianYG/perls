# !/usr/bin/env python

import gym
import gym.spaces as spaces
import numpy as np
import logging
    
import sys, os
import os.path as osp
from os.path import join as pjoin
import pybullet as p


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
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(
            osp.abspath(pjoin(osp.dirname(__file__),
                  '../../../data')))
        p.resetSimulation()

        self._robot = p.loadURDF('sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf',
            (0, 0, 0.9), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        p.loadURDF('plane.urdf')

        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        # Sawyer control rate 200Hz
        p.setTimeStep(0.05)
        self._goal = np.array(goal)

        # self._robot.spin(hz, ctype='velocity')
        self._scale = np.array([p.getJointInfo(self._robot, i)[11] for i in range(7)])
    
    @property
    def action_space(self):
        """
        Get the space of actions in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.array((-1,) * 7),
            high=np.array((1,) * 7))

    @property
    def observation_space(self):
        """
        Get the space of observations in the environment
        :return: Space object
        """
        return spaces.Box(
            low=np.array([-1] * 14 + list(-self._scale)),
            high=np.array([1] * 14 + list(self._scale)))

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
        p.resetSimulation()
        p.disconnect()

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

        jpos = [p.getJointState(self._robot, i)[0] for i in range(7)]
        jvel = [p.getJointState(self._robot, i)[1] for i in range(7)]

        return np.concatenate((
            np.cos(jpos), 
            np.sin(jpos), 
            jvel))

    def _reset(self):
        """
        Reset the world environment.
        :return: The state defined by users constrained by
        state space.
        """
        # Initialize to random position
        rand_start = np.random.uniform((0.233, 0.347, 0), (0.545, 0.177, 0.521))
        # self._robot.tool_pose = (rand_start, (0, 1, 0, 0))

        rest = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]

        for i in range(7):
            p.resetJointState(self._robot, i, targetValue=rest[i])
        p.stepSimulation()
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
        p.setJointMotorControlArray(self._robot, range(7), p.VELOCITY_CONTROL, targetVelocities=tuple(action))
        # 20 hz
        # for _ in range(20):
        p.stepSimulation()
        tool_pos = np.array(p.getLinkState(self._robot, 6)[0])
        done = True if np.allclose(self._goal, tool_pos, rtol=1e-2) else False
        rew = int(done) + np.exp(-2. * np.linalg.norm(self._goal - tool_pos, 2)) - 1
        return self._state, rew, done, \
            {'eef position': tool_pos, 'goal': self._goal}
