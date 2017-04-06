import struct
import math
from bullet.core.vr_physics import BulletVR

class KukaVR(BulletVR):

	def __init__(self, pybullet, task):

		super(KukaVR, self).__init__(pybullet, task)

		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
		self.JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
		self.REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
		self.KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
		self.KUKA_GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, 
			-0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]
		self.THRESHOLD = 1.3
		self.MAX_FORCE = 500

	def create_scene(self):
		"""
		Basic scene needed for running tasks
		"""
		self.p.resetSimulation()
		self.p.setGravity(0, 0, -9.81)
		self.p.loadURDF("plane.urdf",0,0,0,0,0,0,1)
		self.p.loadURDF("table/table.urdf", 1.1, -0.2, 0., 0., 0., 0.707107, 0.707107)
		self._setup_robot()

	def reset_kuka(self, kuka):
		for jointIndex in range(self.p.getNumJoints(kuka)):
			self.p.resetJointState(kuka, jointIndex, self.REST_JOINT_POS[jointIndex])
			self.p.setJointMotorControl2(kuka, jointIndex, self.p.POSITION_CONTROL, 
				self.REST_JOINT_POS[jointIndex], 0)

	def reset_kuka_gripper(self, kuka_gripper):
		for jointIndex in range(self.p.getNumJoints(kuka_gripper)):
			self.p.resetJointState(kuka_gripper, jointIndex, self.KUKA_GRIPPER_REST_POS[jointIndex])
			self.p.setJointMotorControl2(kuka_gripper, jointIndex, 
				self.p.POSITION_CONTROL, self.KUKA_GRIPPER_REST_POS[jointIndex], 0)

	def engage(self, kuka, controller_event, fixed=True):

		controller_pos = controller_event[1]
		controller_orn = controller_event[self.ORIENTATION]
		targetPos = controller_pos
		eef_orn = controller_orn
		if controller_event[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
			if fixed:
				self.ik_helper(kuka, targetPos, (0, 1, 0, 0), fixed=fixed)
			else:
				# eef_orn = self.p.getQuaternionFromEuler([0, -math.pi, z])
				self.ik_helper(kuka, targetPos, controller_orn)

	def disengage(self, kuka, controller_event):

		if controller_event[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
			for jointIndex in range(self.p.getNumJoints(kuka)):
				# self.p.resetJointState(kuka, jointIndex, self.REST_JOINT_POS[jointIndex])
				self.p.setJointMotorControl2(kuka, jointIndex, self.p.POSITION_CONTROL, 
					self.REST_JOINT_POS[jointIndex], 0)

	def ik_helper(self, arm_id, eef_pos, eef_orien, fixed=False):

		if fixed:
			joint_pos = self.p.calculateInverseKinematics(arm_id, 6, eef_pos, eef_orien, 
				lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, 
				jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			for i in range(len(joint_pos)):
				self.p.setJointMotorControl2(arm_id, i, self.p.POSITION_CONTROL, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=self.MAX_FORCE)
		else:
			joint_pos = self.p.calculateInverseKinematics(arm_id, 6, eef_pos, 
				lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, 
				jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
			# Only need links 1- 5, no need for joint 4-6 with pure position IK
			for i in range(len(joint_pos) - 3):
				self.p.setJointMotorControl2(arm_id, i, self.p.POSITION_CONTROL, 
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=self.MAX_FORCE)
			
			x, y, z = self.p.getEulerFromQuaternion(eef_orien)
			# End effector needs protection, done by using triangular tricks

			# if self.LOWER_LIMITS[0] < z < self.UPPER_LIMITS[0]:	# JOInt limits!!
			# 	self.p.setJointMotorControl2(arm_id, 0, self.p.POSITION_CONTROL, 
			# 		targetPosition=z, targetVelocity=0, positionGain=0.02, velocityGain=1, force=self.MAX_FORCE)
			# else:
			# 	self.p.addUserDebugText('Warning: you are flipping arm link 0', self.p.getLinkState(arm_id, 0)[0], 
			# 		textColorRGB=(255, 0, 0), lifeTime=1.5)
			# 	self.p.setJointMotorControl2(arm_id, 0, self.p.POSITION_CONTROL, 
			# 		targetPosition=joint_pos[0], targetVelocity=0, positionGain=0.01, velocityGain=1.0, force=self.MAX_FORCE)

			# self.p.setJointMotorControl2(arm_id, 6, self.p.POSITION_CONTROL, 
			# 	targetPosition=np.arcsin(np.sin(z)), targetVelocity=0, positionGain=0.6, velocityGain=1.0, force=self.MAX_FORCE)
			
			#TO-DO: add wait till fit

			# Link 4 needs protection
			if self.LOWER_LIMITS[6] < x < self.UPPER_LIMITS[6]:	# JOInt limits!!
				self.p.setJointMotorControl2(arm_id, 6, self.p.POSITION_CONTROL, 
					targetPosition=x, targetVelocity=0, positionGain=0.02, velocityGain=1, force=self.MAX_FORCE)
			else:
				self.p.addUserDebugText('Warning: you are flipping arm link 6', self.p.getLinkState(arm_id, 0)[0], 
					textColorRGB=(255, 0, 0), lifeTime=1.5)
				self.p.setJointMotorControl2(arm_id, 6, self.p.POSITION_CONTROL, 
					targetPosition=joint_pos[6], targetVelocity=0, positionGain=0.01, velocityGain=1.0, force=self.MAX_FORCE)

			if self.LOWER_LIMITS[5] < y < self.UPPER_LIMITS[5]:
				self.p.setJointMotorControl2(arm_id, 5, self.p.POSITION_CONTROL, 
					targetPosition=-y, targetVelocity=0, positionGain=0.03, velocityGain=1.0, force=self.MAX_FORCE)
			else:
				self.p.addUserDebugText('Warning: you are flipping arm link 5', self.p.getLinkState(arm_id, 1)[0], 
					textColorRGB=(255, 0, 0), lifeTime=1.5)
				self.p.setJointMotorControl2(arm_id, 5, self.p.POSITION_CONTROL, 
					targetPosition=joint_pos[5], targetVelocity=0, positionGain=0.01, velocityGain=1.0, force=self.MAX_FORCE)

	def euc_dist(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist
