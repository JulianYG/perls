import pybullet as p
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import h5py
from pylab import *
import numpy as np

CONTROLLER_ID = 0
POSITION = 1
BUTTONS = 6

FOCAL_POINT = [0., 0., 0.]
YAW = 40.
# PITCH = 10.
ROLL = 0.
FOCAL_LENGTH = 4.
UP_AX_IDX = 2

# VR physics server must already started
p.connect(p.SHARED_MEMORY)
p.setInternalSimFlags(0)  # don't load default robot assets etc
p.resetSimulation()

def _create_default_assets():

	p.setGravity(0,0,-9.8)
	p.loadURDF("plane.urdf")
	# ob = p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
	p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)

	objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
	ob = objects[0]
	jointPositions=[ -0.000000, 0.000000, -0.000001, 1.570795, 0.000000, -1.036725, 0.000001 ]
	for jointIndex in range (p.getNumJoints(ob)):
		p.resetJointState(ob,jointIndex,jointPositions[jointIndex])


	ob_start = p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	ob_end = p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107)
	

def _setup_robot_arm():



def load_task_hanoi():

	# 3 poles for Tower of Hanoi, fixed base
	p.loadURDF("cvgl/pole.urdf", 1.00000,-0.300000,0.699990,-0.000005,0.707107,0.000006,0.707107,0,1)
	p.loadURDF("cvgl/pole.urdf", 1.00000,-0.500000,0.699990,-0.000005,0.707107,0.000006,0.707107,0,1)
	p.loadURDF("cvgl/pole.urdf", 1.00000,-0.700000,0.699990,-0.000005,0.707107,0.000006,0.707107,0,1)


	ob_start = 

	return range(ob_start, ob_end + 1)
	
def create_task_scene():
	"""
	Setup the default scene
	"""
	_create_default_assets()
	tracking_arm = _setup_robot_arm()
	tracking_objs = load_task_hanoi()
	return tracking_arm, tracking_objs


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


arm, obj = create_task_scene()
ani = render_camera_port(FOCAL_POINT, FOCAL_LENGTH, YAW, PITCH, ROLL, UP_AX_IDX)
writer = animation.writers['ffmpeg'](fps=1000/75)

try:

	while True:
		events = p.getVREvents()
		for e in events:
			pass

		if events[5] > 0:	# Only take record when moving events happen
			for j_id in arm:
				break

			for o_id in obj:
				print(o_id, p.getBasePositionAndOrientation(o_id))

except KeyboardInterrupt:
	
	ani.save('demo.mp4', wirter=writer, dpi=100)




