import os, sys
from os.path import join as pjoin
path = os.path.abspath(os.getcwd()).rsplit('/')
rpath = '/'.join(path[: path.index('perls') + 1])
sys.path.append(pjoin(rpath, 'src/real_'))

from utils.camera import UVCCamera
from utils.uvc_tracker import Tracker
from robot import Robot

import rospy
import intera_interface

rospy.init_node('track')
limb = intera_interface.Limb('right')
limb.set_joint_position_speed(0.2)
robot = Robot(limb, None)

camera = UVCCamera(0, (1280, 720), _UK, _UD)

tracker = Tracker(camera, robot, 
	K=_UK, D=_UD, board_size=(9, 6), itermat=(9, 9), debug=False)

tracker.match_eval()



