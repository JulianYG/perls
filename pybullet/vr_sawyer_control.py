import pybullet as p
import numpy as np

def euc_dist(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist

LOWER_LIMITS = [0, 0, 0, 0, 0, -3.05, -5.1477, 0, 0, 0, -1.57079632679, -3.0514, 
	-3.0514, -2.9842, 0, -2.9842, 0, 0, -2.9842, 0]
UPPER_LIMITS = [0, 0, 0, 0, 0, 3.05, 0.9599, 0, 0, 0, 1.57079632679, 3.0514, 
	3.0514, 2.9842, 0, 2.9842, 0, 0, 2.9842, 0]
JOINT_RANGE = [0, 0, 0, 0, 0, 6.1, 6.1, 0, 0, 0, 6.1, 6.1, 6.1, 5.96, 0, 5.96, 0, 0, 9.4, 0]
REST_POSE = [0, 0, 0, 0, 0, 0.00, 0, 0, 0, 0, -1.18, 0.00, 2.18, 0.00, 0, 0.57, 0, 0, 3.3161, 0]
JOINT_DAMP = [0.1] * 20

# LOWER_LIMITS = [-3.0503, -5.1477, -3.8183, -3.0514, 
# 	-3.0514, -2.9842, -2.9842, -4.7104]
# UPPER_LIMITS = [3.0503, 0.9599, 2.2824, 3.0514, 
# 	3.0514, 2.9842, 2.9842, 4.7104]
# JOINT_RANGE = [6.1, 6.1, 6.1, 6.1, 6.1, 5.96, 5.96, 9.4]
REST_POSE_IK = [0, 0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]

MAX_FORCE = 500

KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.2019421, 0.205143, -0.009999, 0., -0.010055, 0.]
KUKA_GRIPPER_CLOZ_POS = [0.0, -0.04751942419423083795, 0.1985595192347591911, -0.7479294372303137, 0.0505459999199719922, 0.0, 0.0498381051978835724, 0.0]

THRESHOLD = 1.2

p.connect(p.SHARED_MEMORY)
p.setInternalSimFlags(0)
p.setRealTimeSimulation(1)
p.resetSimulation()

p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
sawyer = p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0.5, -0.3,
	0], useFixedBase=True)

p.resetBasePositionAndOrientation(sawyer, [0.5, -1, 0.912989899704], [0, 0, 0, 1])

for jointIndex in range (p.getNumJoints(sawyer)):
	qIndex = p.getJointInfo(sawyer, jointIndex)[3]
	if qIndex > -1:
		p.resetJointState(sawyer,jointIndex,REST_POSE[jointIndex])
		p.setJointMotorControl2(sawyer, jointIndex, p.POSITION_CONTROL, 
			targetPosition=REST_POSE[qIndex - 7], targetVelocity=0, 
				positionGain=0.05, velocityGain=1.0, force=MAX_FORCE)

objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
sawyer_gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]

print("sawyer gripper={}".format(sawyer_gripper))

p.resetBasePositionAndOrientation(sawyer_gripper,[0.923103,-0.200000,1.2500319],[-0.000000,0.9194531,-0.000002,-0.2193970])
jointPositions=[ 0.000000, -0.011130, -0.2019421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
for jointIndex in range (p.getNumJoints(sawyer_gripper)):
	p.resetJointState(sawyer_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(sawyer_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

sawyer_cid = p.createConstraint(sawyer, 19, sawyer_gripper, 0, p.JOINT_FIXED, 
	[0,0,0], [0,0,0.05],[0,0,0])

p.setGravity(0,0,-9.81)

# objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
# objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
# objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
# objects = p.loadSDF("kiva_shelf/model.sdf")
# ob = objects[0]
# p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
ball = p.loadURDF("sphere_small.urdf", -0.100000,0.9550019,1.11997019,0.1933232,-0.000000,-0.000000,0.7739192)

(0, 'controller_box_fixed', 4, -1, -1, 0, 0.0, 0.0)
(1, 'pedestal_feet_fixed', 4, -1, -1, 0, 0.0, 0.0)
(2, 'torso_t0', 4, -1, -1, 0, 0.0, 0.0)
(3, 'pedestal_fixed', 4, -1, -1, 0, 0.0, 0.0)
(4, 'right_arm_mount', 4, -1, -1, 0, 0.0, 0.0)
(5, 'right_j0', 0, 7, 19, 1, 0.0, 0.0)
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
(18, 'right_j19', 0, 14, 13, 1, 0.0, 0.0)
(19, 'right_hand', 4, -1, -1, 0, 0.0, 0.0)

a = p.createConstraint(sawyer, -1, sawyer, 0, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
b = p.createConstraint(sawyer, -1, sawyer, 1, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
c = p.createConstraint(sawyer, -1, sawyer, 2, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
d = p.createConstraint(sawyer, -1, sawyer, 3, p.JOINT_FIXED,[0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
e = p.createConstraint(sawyer, -1, sawyer, 4, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000],
	[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])

# p.createConstraint(sawyer, 5, sawyer, 9, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 19, sawyer, 7, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 19, sawyer, 8, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 13, sawyer, 14, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 15, sawyer, 16, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 15, sawyer, 17, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
# p.createConstraint(sawyer, 18, sawyer, 19, p.JOINT_FIXED, [0.000000,0.000000,0.000000],
# 	[0.000000,0.00000,0.00000],[0.000000,0.00000,0.00000])
POSITION = 1
ORIENTATION = 2
BUTTONS = 6

controllers = [e[0] for e in p.getVREvents()]

while True:

	for e in (p.getVREvents()):

		# Only use one controller
		###########################################
		# This is important: make sure there's only one VR Controller!
		# if e[0] == controllers[0]:
		# 	break

		# A simplistic version of gripper control
		#@TO-DO: Add slider for the gripper

		if e[BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			# avg = 0.
			for i in range(p.getNumJoints(sawyer_gripper)):
				p.setJointMotorControl2(sawyer_gripper, i, p.POSITION_CONTROL, targetPosition=KUKA_GRIPPER_CLOZ_POS[i], force=50)

		if e[BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(sawyer_gripper)):
				p.setJointMotorControl2(sawyer_gripper, i, p.POSITION_CONTROL, targetPosition=KUKA_GRIPPER_REST_POS[i], force=50)

		eef_pos = e[POSITION]
		sq_len = euc_dist(p.getLinkState(sawyer, 19)[0], eef_pos)

		if sq_len < THRESHOLD * THRESHOLD:
			
			joint_pos = p.calculateInverseKinematics(sawyer, 19, eef_pos, (0, 1, 0, 0),
				lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
				jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

			for i in range(p.getNumJoints(sawyer)):
				qIndex = p.getJointInfo(sawyer, i)[3]
				print(qIndex, i)
				if qIndex > 0:
					p.setJointMotorControl2(sawyer, i, p.POSITION_CONTROL, 
						targetPosition=joint_pos[qIndex - 7], targetVelocity=0, positionGain=0.05, 
						velocityGain=1.0, force=MAX_FORCE)
	
			# Rotate the end effector
			targetOrn = e[ORIENTATION]

			_, _, z = p.getEulerFromQuaternion(targetOrn)
			# End effector needs protection, done by using triangular tricks
			print(joint_pos)

			# if LOWER_LIMITS[19] < z < UPPER_LIMITS[19]:
			# 	p.setJointMotorControl2(sawyer, 19, p.POSITION_CONTROL, 
			# 		targetPosition=z, targetVelocity=0, positionGain=0.03, velocityGain=1.0, force=MAX_FORCE)


			# else:
			# 	p.setJointMotorControl2(sawyer, 19, p.POSITION_CONTROL, 
			# 		targetPosition=joint_pos[19], targetVelocity=0, positionGain=0.05, 
			# 		velocityGain=1.0, force=MAX_FORCE)

		else:
			# Set back to original rest pose
			for jointIndex in range(p.getNumJoints(sawyer)):
				p.setJointMotorControl2(sawyer, jointIndex, p.POSITION_CONTROL, 
					REST_POSE[jointIndex], 0)

