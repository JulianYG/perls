
import redis
import sys, os, subprocess
from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'ros_'))
# os.system('./ros_/run.sh')
# subprocess.call(['cd', '~/ros_ws'])
# subprocess.call('./ros_/intera.sh')
# subprocess.call('cd {}'.format(os.getcwd()))

import rospy
import intera_interface
from robot import Robot

'''
Before initializing an instance of the Robot_Control class:
	rospy.init_node("sdk_wrapper") #initializes node on machine to talk to arm
	limb = intera_interface.Limb('right')
	gripper = intera_interface.Gripper('right')
	arm = Robot_Control(limb, gripper)
	
'''
rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

import pybullet as p
import Queue, time
import numpy as np
event_queue = Queue.Queue(4096)

def _event_handler(msg):
	if not event_queue.full():
		event_queue.put(eval(msg['data']))

r = redis.StrictRedis(host='localhost', port=6379, db=0)
pubsub = r.pubsub()
pubsub.subscribe(**{'event_channel': _event_handler})

# Start thread
client_thread = pubsub.run_in_thread(sleep_time=0.001)

arm = Robot(limb, gripper)

# Initializing
rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
	'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}
arm.set_init_positions(rest_pose)
arm.slide_grasp(1)

arm_initial_pos = np.array(list(arm.get_tool_pose()[0]))

vr_initial_pos = None

while True:
	if not event_queue.empty():
		vr_initial_pos = np.array(list(event_queue.get()[1]))
		break


FILTER_SIZE = 20
lpf_list = []
lpf_num_elems = 0

while True:
	try:
		while not event_queue.empty():
			e = event_queue.get()
			pos, orn = e[1], e[2]
			if e[6][33] & p.VR_BUTTON_WAS_TRIGGERED:
				arm.slide_grasp(0)
			if e[6][33] & p.VR_BUTTON_WAS_RELEASED:
				arm.slide_grasp(1)

			if e[6][1] & p.VR_BUTTON_WAS_TRIGGERED:
				arm.set_init_positions(rest_pose)
				vr_initial_pos = np.array(list(e[1]))

			arm_rel_pos = arm.get_relative_pose()['position']

			rel_pose = np.array([arm_rel_pos.x ,arm_rel_pos.y, arm_rel_pos.z])

			vr_rel_pos = np.array(list(pos) - vr_initial_pos)

			target_pos = arm_initial_pos + vr_rel_pos

			if lpf_num_elems >= FILTER_SIZE:
				lpf_list.pop(0)
				lpf_list.append(target_pos)
			else:
				lpf_list.append(target_pos)
				lpf_num_elems += 1

			target_pos = np.mean(np.array(lpf_list), axis=0)

			if 0.1 < np.sqrt(np.sum(vr_rel_pos ** 2)) < 0.8:
				if not arm.reach_absolute({'position': target_pos, 'orientation': (0, 1, 0, 0)}):
					target_pos = arm_initial_pos

	except KeyboardInterrupt:
		client_thread.stop()
		pubsub.unsubscribe()
		arm.shutdown()
		sys.exit(0)




# print(initial_pos)
# pose = {'position': (-0.605, 0.076, 0.86), 'orientation': (0, 1, 0, 0)}

# arm.reach(pose)

