import ast
import openvr

from pybullet import (getKeyboardEvents,
					  getVREvents,
					  getMouseEvents)
from .math_util import vec, mat4, mat2pose


_X_POS_VEC = vec((.001, .0, .0))
_X_NEG_VEC = vec((-.001, .0, .0))

_Y_POS_VEC = vec((.0, .001, .0))
_Y_NEG_VEC = vec((.0, -.001, .0))

_Z_POS_VEC = vec((.0, .0, .001))
_Z_NEG_VEC = vec((.0, .0, -.001))

DEVICE_TYPE = dict(controller=2,
				   head_mount=1,
				   generic=3,
				   light_house=4)

KEY_STATUS = {0: 'idle',
			  1: 'holding',
			  2: 'triggered',
			  3: 'pressing',
			  4: 'releasing',
			  5: 'error',
			  6: 'pre-pressed',
			  7: 'invalid'
			  }

KEY_LABEL = {65284: 'rst', # F5 for reset
			 65306: 'cam', # for camera control, SHIFT
			 65307: 'tbd', # reserved, CONTROL

			 # 6dof Orientation control
			 65295: 'orn',  # 'roll_counterclockwise', <--
			 65296: 'orn',  # 'roll_clockwise', -->
			 65297: 'orn',  # 'pitch_up', ^
			 65298: 'orn',  # 'pitch_down', v
			 114: 'orn',  # 'yaw_left', R
			 102: 'orn',  # 'yaw_right, F

			 # 6dof Position control
			 119: 'pos',  # 'x_forward', W
			 115: 'pos',  # 'x_backward', S
			 97: 'pos',   # 'y_left', A
			 100: 'pos',   # 'y_right', D
			 101: 'pos',   # 'z_up', E
			 113: 'pos',   # 'z_down', Q

			 # Tool mega-control
			 32: 'grasp',  # Space
			 103: 'key',   # 'gripper', G
			 109: 'key',   # 'arm', M
			 44: 'tool',   # 'last_tool', ','
			 46: 'tool',   # 'next_tool', '.'
			 # 48 - 57 corresponds to 'id' type, 0-9
			 }

HOT_KEY = {65284: None,  # F5
		   65306: None,  # SHIFT
		   65307: None,  # CONTROL

		   # Tool Orientation control
		   65295: _X_NEG_VEC,  # <^
		   65296: _X_POS_VEC,  # ^>
		   65297: _Y_POS_VEC,  # ^
		   65298: _Y_NEG_VEC,  # v
		   113: _Z_NEG_VEC,  # <--
		   101: _Z_POS_VEC,  # -->

		   # Tool Position control
		   32: None,  # Space
		   119: _X_NEG_VEC,  # W
		   115: _X_POS_VEC,  # S
		   97: _Y_NEG_VEC,  # A
		   100: _Y_POS_VEC,  # D
		   114: _Z_POS_VEC,  # R
		   102: _Z_NEG_VEC,  # F
		   103: 'g',
		   109: 'm',
		   44: -1,  # ,
		   46: 1,  # .
		   # 48 - 57 corresponds to their own values - 48.
		   }


def listen_to_bullet_keyboard(ps_id=0):
	return getKeyboardEvents(physicsClientId=ps_id)


def listen_to_bullet_mouse(ps_id=0):
	return getMouseEvents(physicsClientId=ps_id)


def listen_to_bullet_vive(vr_system, vr_compositor, *dtype):
	t = 0
	for device in dtype:
		t |= DEVICE_TYPE[device]

	# _, state, pose = vr_system.getControllerStateWithPose(
	#     openvr.TrackingUniverseStanding, 
	#     5, 
	#     1)
	events = dict()
	poses = vr_system.getDeviceToAbsoluteTrackingPose(
		openvr.TrackingUniverseStanding,
		0,
		openvr.k_unMaxTrackedDeviceCount)

	for device_idx in range(openvr.k_unMaxTrackedDeviceCount):

	  	device_pose = poses[device_idx]

	  	if device_pose.bDeviceIsConnected:

			device_type = vr_system.getTrackedDeviceClass(device_idx)

		if device_type == t & 1:
		  	events['hmd'] = dict(c_id=device_idx, pose=poses[device_idx])
		elif device_type == t & 2:
		  	events['controller'] = dict(c_id=device_idx, pose=poses[device_idx], events=)

		elif device_type == t & 3:
		  	events['generic'] = dict(c_id=device_idx, pose=poses[device_idx])
		elif device_type == t & 4:
		 	events['lighthouse'] = dict(c_id=device_idx, pose=poses[device_idx])

		



	pose = poses[5]
	vr_compositor.waitGetPoses(poses, openvr.k_unMaxTrackedDeviceCount, None, 0)  

	# print(vr_system.pollNextEvent(openvr.TrackingUniverseStanding))
	# print(pose.bDeviceIsConnected, dir(state.rAxis), state.ulButtonPressed, state.ulButtonTouched, state.unPacketNum)
	# print([vr_system.getTrackedDeviceClass(i) for i in range(16)])
	# print(dir(state), dir(pose), pose.mDeviceToAbsoluteTracking)
	# return (getVREvents(deviceTypeFilter=t,
	#                     physicsClientId=ps_id))
	pos, orn = mat2pose(
	  	mat4([
			list(pose.mDeviceToAbsoluteTracking[0]),
			list(pose.mDeviceToAbsoluteTracking[1]),
			list(pose.mDeviceToAbsoluteTracking[2]),
			[0, 0, 0, 1]
	  	])
	)
	return 5, tuple(pos[[0, 2, 1]]), tuple(orn)


def listen_to_redis(queue):
	"""
	Listen to a connected and subscribed redis queue
	:param queue: the registered queue that
	receives data from callback function
	:return: 
	"""
	# TODO: construct socket class
	events = list()
	while not queue.empty():
		item = queue.get()
		# parses into AST (Source Tree),
		# only evaluates/returns if it's literal,
		# so no safety issue
		signal_dic = ast.literal_eval(item)
		events.append(signal_dic)
	return events

