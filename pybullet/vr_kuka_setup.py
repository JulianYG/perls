import pybullet as p
import math
p.connect(p.SHARED_MEMORY)

p.setInternalSimFlags(0)

p.resetSimulation()

objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
pr2_gripper = objects[0]
print ("pr2_gripper=")
print (pr2_gripper)

jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
for jointIndex in range (p.getNumJoints(pr2_gripper)):
	p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])

pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000])
print ("pr2_cid")
print (pr2_cid)

objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
kuka = objects[0]
jointPositions=[ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
for jointIndex in range (p.getNumJoints(kuka)):
	p.resetJointState(kuka,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.800000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.900000,0.000000,0.000000,0.000000,1.000000)]
objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
kuka_gripper = objects[0]
print ("kuka gripper=")
print(kuka_gripper)

p.resetBasePositionAndOrientation(kuka_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
jointPositions=[ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
for jointIndex in range (p.getNumJoints(kuka_gripper)):
	p.resetJointState(kuka_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)


# kuka_cid = p.createConstraint(kuka,   6,  kuka_gripper, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])

objects = [p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
objects = p.loadSDF("kiva_shelf/model.sdf")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
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

p.setGravity(0,0,-10)

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
BUTTONS=6

def euc_dist(posA, posB):

	dist = 0.
	# print (posA)
	# print (posB)
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2

	return dist

LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]

# print(p.getNumJoints(kuka_gripper))
while True:
	events = p.getVREvents()
	# print(p.getLinkState(kuka, 6))
	# for e in (events):
	# 	if (e[BUTTONS][33]&p.VR_BUTTON_IS_DOWN):
	# 		p.changeConstraint(kuka_cid,e[POSITION],e[ORIENTATION], maxForce=50)
			#todo
			# p.setJointMotorControl2(pr2_gripper,0, p.POSITION_CONTROL)
	for e in (events):
		
		# for i in range(64):
		# 	if e[BUTTONS][i] & p.VR_BUTTON_WAS_TRIGGERED:
		# 		print(i)
	
		if e[BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			for i in range(p.getNumJoints(kuka_gripper)):
				p.setJointMotorControl2(kuka_gripper, i, p.VELOCITY_CONTROL, targetVelocity=5, force=50)
		
			# p.setJointMotorControl2(kuka_gripper, 6, p.VELOCITY_CONTROL, targetVelocity=5, force=5)
		
			# p.setJointMotorControl2(kuka_gripper, 4, p.VELOCITY_CONTROL, targetVelocity=5, force=5)
		if e[BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:
				
			for i in range(p.getNumJoints(kuka_gripper)):
				p.setJointMotorControl2(kuka_gripper, i, p.VELOCITY_CONTROL, targetVelocity=-5, force=50)
		
			# p.setJointMotorControl2(kuka_gripper, 6, p.VELOCITY_CONTROL, targetVelocity=-5, force=5)
		
			# p.setJointMotorControl2(kuka_gripper, 4, p.VELOCITY_CONTROL, targetVelocity=-5, force=5)

		# print((p.getBasePositionAndOrientation(kuka_gripper)[0], e[1]))
		sq_len = euc_dist(p.getBasePositionAndOrientation(kuka_gripper)[0], e[1])
		threshold = 1.15
		
		if sq_len < threshold * threshold:
			# time = 0.0
			# time += 0.01
			# targetPos = (0.4 - 0.4 * math.cos(time), 0, 0.8 + 0.4 * math.cos(time))
			# eef_pos = (e[1][0] + targetPos[0], e[1][1] + targetPos[1], e[1][2] + targetPos[2])
			targetPos = e[1]
			
			x, y, z_orig = p.getEulerFromQuaternion((0, 1, 0, 0))
			_, _, z = p.getEulerFromQuaternion(e[ORIENTATION])
			# # print(x, y, z)
			# eef_orien = p.getBasePositionAndOrientation(kuka_gripper)[1]
			eef_orien = p.getQuaternionFromEuler([x, y, z])
			# print(eef_orien)

			print(p.VR_BUTTON_IS_DOWN)
			if e[BUTTONS][32] & p.VR_BUTTON_WAS_RELEASED:
			
				_, _, z = p.getEulerFromQuaternion(e[ORIENTATION])
				p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
			if e[BUTTONS][32] & p.VR_BUTTON_IS_DOWN:
					
				p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z_orig, force=5)
				joint_pos = p.calculateInverseKinematics(kuka, 6, targetPos, (0, 1, 0, 0), lowerLimits=LOWER_LIMITS, 
					upperLimits=UPPER_LIMITS, jointRanges=JOINT_RANGE, restPoses=REST_POSE)
				for i in range(len(joint_pos)):
					p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL, targetPosition=joint_pos[i], force=500)

			
			# p.resetBasePositionAndOrientation(kuka_gripper, p.getBasePositionAndOrientation(kuka_gripper)[0], eef_orien)

			# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
			# if e[BUTTONS][32] & p.VR_BUTTON_WAS_TRIGGERED:

				# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
		else:
			jointPositions=[ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
			for jointIndex in range (p.getNumJoints(kuka)):
				p.setJointMotorControl2(kuka,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex], 0)
			# 


p.stepSimulation()

p.disconnect()
