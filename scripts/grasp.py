#!/usr/bin/env python

import time, os
import sys
import numpy as np

from os.path import join as pjoin

sys.path.append(os.path.abspath(pjoin(os.path.dirname(__file__), '../src/ros_')))

from utils.uvc_tracker import Tracker

from robot import Robot

rospy.init_node('track')
limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, None)

camera = UVCCamera(0, (1280, 720), _UK, _UD)

tracker = Tracker(camera, robot, 
	K=_UK, D=_UD, board_size=(9, 6), itermat=(9, 9), debug=False)

tracker.match_eval()


