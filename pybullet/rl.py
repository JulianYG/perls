import pybullet as p
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import h5py
from pylab import *
import numpy as np
import time
import csv
import math

CONTROLLER_ID = 0
KUKA_GRIPPER_ID = 3
POSITION = 1
BUTTONS = 6

FOCAL_POINT = [0., 0., 0.]
YAW = 40.
PITCH = 0.
ROLL = 0.
FOCAL_LENGTH = 4.
UP_AX_IDX = 2

# VR physics server must already started
p.connect(p.SHARED_MEMORY)
# p.setInternalSimFlags(0)  # don't load default robot assets etc
p.resetSimulation()

def _create_default_assets():

	p.setGravity(0,0,-9.8)
	# ob = p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
	p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)

	# objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
	# ob = objects[0]
	# jointPositions=[ -0.000000, 0.000000, -0.000001, 1.570795, 0.000000, -1.036725, 0.000001 ]
	# for jointIndex in range (p.getNumJoints(ob)):
	# 	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

	p.loadURDF("jenga/jenga.urdf", 1.300000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 1.200000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 1.100000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)

def _setup_robot_arm():

	# robot = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.4,-0.2,0.6,0,0,0,1)
	# joint_pos_angle = [0,0,0,math.pi/2,0,-math.pi/2*0.66,0]
	# for i in range(len(joint_pos_angle)):
	# 	p.resetJointState(robot, i, joint_pos_angle[i])
	# gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")

	# return robot
	pass

def load_task_hanoi():

	# 3 poles for Tower of Hanoi, fixed base
	p.loadURDF("rl/pole.urdf",0.80000,0.100000,0.699990,0.00000,0.0,0.00000,1)
	p.loadURDF("rl/pole.urdf",0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1)
	p.loadURDF("rl/pole.urdf",0.80000,-0.500000,0.699990,0.00000,0.0,0.00000,1)

	ob_start = p.loadURDF("rl/torus_0.urdf",0.8,0.1,0.69999,1,0,0,1)
	p.loadURDF("rl/torus_1.urdf",0.8,0.1,0.74999,1,0,0,1)
	p.loadURDF("rl/torus_2.urdf",0.8,0.1,0.79999,1,0,0,1)
	p.loadURDF("rl/torus_3.urdf",0.8,0.1,0.84999,1,0,0,1)
	ob_end = p.loadURDF("rl/torus_4.urdf",0.8,0.1,0.89999,0,0,0,1)

	return range(ob_start, ob_end + 1)
	
def create_task_scene():
	"""
	Setup the default scene
	"""
	_create_default_assets()
	# robot = _setup_robot_arm()
	tracking_objs = load_task_hanoi()
	return tracking_objs


def set_camera_position():
	# upAxisIndex: 1 for Y, 2 for Z
	pass

def render_scene_full(intvl=10):
	for pitch in range(0, 360, intvl):
		viewMatrix = p.computeViewMatrixFromYawPitchRoll(FOCAL_POINT, FOCAL_LENGTH, YAW, pitch, ROLL, UP_AX_IDX)
		projectionMatrix = p.computeProjectionMatrixFOV(60, 320 / 240., 0.01, 1000.)
		img_arr = p.getCameraImage(320, 240, viewMatrix, projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.
		plt.imshow(np_img)
		plt.pause(0.01)

def render_camera_port(focus_pt, focal_len, yaw, pitch, roll, upAxisIndex=2):
	viewMatrix = p.computeViewMatrixFromYawPitchRoll(focus_pt, focal_len, yaw, pitch, roll, upAxisIndex)
	projectionMatrix = p.computeProjectionMatrixFOV(60, 320/240., 0.01, 1000.)
	img_arr = p.getCameraImage(320, 240, viewMatrix, projectionMatrix)
	np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.
	fig = plt.figure()
	ax = fig.add_subplot(111)
	im = ax.imshow(np_img)
	
	def next_camera_frame():
		return p.getCameraImage(320, 240, viewMatrix, projectionMatrix)

	return animation.FuncAnimation(fig, next_camera_frame, interval=75) # draw/75ms


obj = create_task_scene()
ani = render_camera_port(FOCAL_POINT, FOCAL_LENGTH, YAW, PITCH, ROLL, UP_AX_IDX)
# writer = animation.writers['ffmpeg'](fps=1000/75)

try:
	f = open('trajectory' + time.strftime("%m%d-%H%M%S") + '.csv', 'w', newline='')
	writer = csv.writer(f)
	prev_time = time.time()
	while True:
		events = p.getVREvents()
		# for e in events:
		# 	pass
		if events and events[0][5] > 0:	# Only take record when moving events happen
			curr_time = time.time()
			time_elapse = curr_time - prev_time
			prev_time = curr_time
			for o_id in obj:
				row = [(o_id)] + list(p.getBasePositionAndOrientation(o_id)[0]) + list(p.getBasePositionAndOrientation(o_id)[1])
				writer.writerow(row)
			gripper_info = [KUKA_GRIPPER_ID] + list(p.getBasePositionAndOrientation(KUKA_GRIPPER_ID)[0]) + \
				list(p.getBasePositionAndOrientation(KUKA_GRIPPER_ID)[1])
			delay = [-1, time_elapse]
			writer.writerow(gripper_info)
			writer.writerow(delay)
except KeyboardInterrupt:
	
	# ani.save('demo.mp4', wirter=writer, dpi=100)
	p.disconnect()
	f.close()


