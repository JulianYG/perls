## Assume you have run vr_kuka_setup and have default scene set up
# Require p.setInternalSimFlags(0) in kuka_setup
import pybullet as p
import math
# import numpy as np

p.connect(p.GUI)

kuka = p.loadURDF('kuka_iiwa/model_vr_limits.urdf')
POSITION = 1
ORIENTATION = 2
BUTTONS = 6

THRESHOLD = 1.3
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500
KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
KUKA_GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, -0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]


# for jointIndex in range(p.getNumJoints(kuka)):
# 	p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, 
# 		REST_JOINT_POS[jointIndex], 0)


def euc_dist(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist

p.setRealTimeSimulation(1)

controllers = [e[0] for e in p.getVREvents()]

while True:

	# Only need links 1- 4, no need for joint 5-6 with pure position IK
	for i in range(7):
		p.setJointMotorControl2(kuka, i, p.TORQUE_CONTROL, force=-500)


	

