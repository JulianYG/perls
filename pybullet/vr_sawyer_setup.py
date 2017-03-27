import pybullet as p
import numpy as np
import math
from datetime import datetime
# p.connect(p.SHARED_MEMORY)

LOWER_LIMITS = [0, 0, 0, 0, 0, -3.05, -5.1477, 0, 0, 0, -1.57079632679, -3.0514, 
	-3.0514, -2.9842, 0, -2.9842, 0, 0, -2.9842, 0]
UPPER_LIMITS = [0, 0, 0, 0, 0, 3.05, 0.9599, 0, 0, 0, 1.57079632679, 3.0514, 
	3.0514, 2.9842, 0, 2.9842, 0, 0, 2.9842, 0]
JOINT_RANGE = [0, 0, 0, 0, 0, 6, 6, 0, 0, 0, 3.14, 6, 6, 5.9, 0, 5.9, 0, 0, 5.9, 0]
REST_POSE = [0, 0, 0, 0, 0, 0.00, 0, 0, 0, 0, -1.18, 0.00, 2.18, 0.00, 0, 0.57, 0, 0, 3.3161, 0]
JOINT_DAMP = [0.1] * 20

MAX_FORCE = 500

p.connect(p.GUI)
p.setRealTimeSimulation(1)

objects = [p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)]
objects = [p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0.0, 0.0,
	0], useFixedBase=True)]
sawyer = objects[0]
p.resetBasePositionAndOrientation(sawyer, [0, 0, 0.912989899704], [0, 0, 0, 1])
jointPositions=[0, 0, 0, 0, 0, 0.00, 0, 0, 0, 0, -1.18, 0.00, 2.18, 0.00, 0, 0.57, 0, 0, 3.3161, 0]

for jointIndex in range (p.getNumJoints(sawyer)):
	p.resetJointState(sawyer,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(sawyer,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.800000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.900000,0.000000,0.000000,0.000000,1.000000)]
objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
sawyer_gripper = objects[0]
print ("sawyer gripper=")
print(sawyer_gripper)

p.resetBasePositionAndOrientation(sawyer_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
jointPositions=[ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
for jointIndex in range (p.getNumJoints(sawyer_gripper)):
	p.resetJointState(sawyer_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(sawyer_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

sawyer_cid = p.createConstraint(sawyer, 19, sawyer_gripper, 0, p.JOINT_FIXED, 
	[0,0,0], [0,0,0.05],[0,0,0])

p.setGravity(0,0,-9.81)

objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
# objects = p.loadSDF("kiva_shelf/model.sdf")
# ob = objects[0]
# p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
objects = [p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)]
objects = [p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

objects = [p.loadURDF("husky/husky.urdf", 2.000000,-5.000000,1.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

# LOWER_LIMITS = [-3.05, -5.1477, -1.57079632679, -3.0514, 
# 	-3.0514, -2.9842, -2.9842, -2.9842]
# UPPER_LIMITS = [3.05, 0.9599, 1.57079632679, 3.0514, 
# 	3.0514, 2.9842, 2.9842, 2.9842]
# JOINT_RANGE = [6.1, 6.1, math.pi, 6.1, 6.1, 5.96, 5.96, 5.96]
# REST_POSE = [0, 0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
# JOINT_DAMP = [0.1] * 8

# for jointIndex in range(p.getNumJoints(sawyer)):
# 	jointInfo = p.getJointInfo(sawyer, jointIndex)
# 	# If flag
# 	if jointInfo[5]:
# 		p.setJointMotorControl2(sawyer, jointInfo[3] - 7, p.POSITION_CONTROL, 
# 			targetPosition=REST_POSE[jointIndex], targetVelocity=0, positionGain=0.03,
# 			velocityGain=1.0, force=MAX_FORCE)

(0, 'controller_box_fixed', 4, -1, -1, 0, 0.0, 0.0)
(1, 'pedestal_feet_fixed', 4, -1, -1, 0, 0.0, 0.0)
(2, 'torso_t0', 4, -1, -1, 0, 0.0, 0.0)
(3, 'pedestal_fixed', 4, -1, -1, 0, 0.0, 0.0)
(4, 'right_arm_mount', 4, -1, -1, 0, 0.0, 0.0)
(5, 'right_j0', 0, 7, 6, 1, 0.0, 0.0)
(6, 'head_pan', 0, 8, 7, 1, 0.0, 0.0)
(7, 'display_joint', 4, -1, -1, 0, 0.0, 0.0)
(8, 'head_camera', 4, -1, -1, 0, 0.0, 0.0)
(9, 'right_torso_itb', 4, -1, -1, 0, 0.0, 0.0)
(10, 'right_j1', 0, 9, 8, 1, 0.0, 0.0)
(11, 'right_j2', 0, 10, 9, 1, 0.0, 0.0)
(12, 'right_j3', 0, 11, 10, 1, 0.0, 0.0)
(13, 'right_j4', 0, 12, 11, 1, 0.0, 0.0)
(14, 'right_arm_itb', 4, -1, -1, 0, 0.0, 0.0)
(15, 'right_j5', 0, 13, 12, 1, 0.0, 0.0)
(16, 'right_hand_camera', 4, -1, -1, 0, 0.0, 0.0)
(17, 'right_wrist', 4, -1, -1, 0, 0.0, 0.0)
(18, 'right_j6', 0, 14, 13, 1, 0.0, 0.0)
(19, 'right_hand', 4, -1, -1, 0, 0.0, 0.0)


p.createConstraint(sawyer, -1, sawyer, 0, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, -1, sawyer, 1, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, -1, sawyer, 2, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, -1, sawyer, 3, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, -1, sawyer, 4, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, 5, sawyer, 6, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
# p.createConstraint(sawyer, 6, sawyer, 7, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
# p.createConstraint(sawyer, 6, sawyer, 8, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.createConstraint(sawyer, 5, sawyer, 9, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.0000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])

# p.createConstraint(sawyer, 13, sawyer, 14, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
# p.createConstraint(sawyer, 15, sawyer, 16, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
# p.createConstraint(sawyer, 15, sawyer, 17, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
# p.createConstraint(sawyer, 18, sawyer, 19, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.000000],[0.000000,0.00000,0.000000],
# 	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])

prevPose=[0,0,0]
prevPose1=[0,0,0]
hasPrevPose = 0
# print(p.getLinkState(sawyer, 19)[1])
# p.setJointMotorControl2(sawyer, 10, p.POSITION_CONTROL, 
# 	targetPosition= -math.pi / 2, targetVelocity=0, positionGain=0.03, velocityGain=1.0,
# 	force=MAX_FORCE)
pos = [0.67, 0.2, 1.4]
p.addUserDebugLine(pos,[0.6, 0.2, 1.5],[1,0,0],100,0)
orn = (0.707, 0.707, 0, 0)
joint_pos = p.calculateInverseKinematics(sawyer, 19, pos, orn,
		lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
		jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

for jointIndex in range(p.getNumJoints(sawyer)):
	qIndex = p.getJointInfo(sawyer, jointIndex)[3]
	if qIndex > -1:
		p.setJointMotorControl2(sawyer, jointIndex, p.POSITION_CONTROL, 
			targetPosition=joint_pos[qIndex - 7], targetVelocity=0, 
			positionGain=0.03, velocityGain=1.0, force=MAX_FORCE)
print(joint_pos)
print (p.getLinkState(sawyer, 19)[4], pos)
while True:

	dt = datetime.now()
	t = (dt.second/60.)*2.*math.pi
	# pos = [0.5, 0.2*math.cos(t),1.2+0.2*math.sin(t)]
	pos = [0.6, 0.2, 1.4]
	p.addUserDebugLine(pos,[0.6, 0.2, 1.5],[1,0,0],100,0)
	orn = (0.707, 0.707, 0, 0)

	# joint_pos = p.calculateInverseKinematics(sawyer, 19, pos, orn,
	# 	lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
	# 	jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

	# for jointIndex in range(p.getNumJoints(sawyer)):
	# 	qIndex = p.getJointInfo(sawyer, jointIndex)[3]
	# 	if qIndex > -1:
	# 		p.setJointMotorControl2(sawyer, jointIndex, p.POSITION_CONTROL, 
	# 			targetPosition=joint_pos[qIndex - 7], targetVelocity=0, 
	# 			positionGain=0.03, velocityGain=1.0, force=MAX_FORCE)

	ls = p.getLinkState(sawyer, 19)
	if hasPrevPose:
		p.addUserDebugLine(prevPose, pos, [0,0,0.3], 1, 60)
		p.addUserDebugLine(prevPose1, ls[4], [1,0,0], 1, 60)
	prevPose=pos
	prevPose1=ls[4]
	hasPrevPose=1
	p.stepSimulation()
