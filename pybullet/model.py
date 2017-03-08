import struct
import math
import time

class BulletPhysicsVR(object):

	def __init__(self, pybullet, task):
		"""
		Other subclasses may re-implement the constructor
		"""
		self.task = task
		self.p = pybullet
		
		self.BUTTONS = 6
		self.ORIENTATION = 2
		self.controllers = None

		# Default settings for camera
		self.FOCAL_POINT = (0., 0., 0.)
		self.YAW = 35.
		self.PITCH = 50.
		self.ROLL = 0.
		self.FOCAL_LENGTH = 5.
		self.UP_AX_IDX = 2
		self.viewMatrix = None
		self.projectionMatrix = None

		self.hand = False
		self.VR_HAND_ID = None

	def setup(self, flag):
		"""
		Load task for both recording and replay
		"""
		try:
			# Use GUI and turn off simulation for replay
			if flag:
				self.p.connect(self.p.GUI)
				self.p.setRealTimeSimulation(0)
				# In order to generate deterministic paths
			else:
				self.p.connect(self.p.SHARED_MEMORY)
				self.p.setRealTimeSimulation(1)
			
			self.p.setInternalSimFlags(0)
			# convenient for video recording
		
		except self.p.error:
			return 0
		self.controllers = [e[0] for e in self.p.getVREvents()]
		self.create_scene()
		for obj in self.task:
			self.p.loadURDF(*obj)
		return 1

	def create_scene(self):
		raise NotImplementedError("Each VR Setup must re-implement this method.")

	def record(self, file):
		raise NotImplementedError("Each VR Setup must re-implement this method.")

	def replay(self, file, saveVideo=0):
		raise NotImplementedError("Each VR Setup must re-implement this method.")

	def set_camera_view(self, targetPosX, targetPosY, targetPosZ, roll, pitch, yaw, dist):
		"""
		Set the view of camera; typically egocentric, or oblique
		"""
		self.FOCAL_POINT = (targetPosX, targetPosY,  targetPosZ)
		self.PITCH = pitch
		self.ROLL = roll
		self.YAW = yaw
		self.FOCAL_LENGTH = dist
		self.viewMatrix = self.p.computeViewMatrixFromYawPitchRoll((targetPosX, targetPosY, targetPosZ), 
			dist, yaw, pitch, roll, self.UP_AX_IDX)
		self.projectionMatrix = self.p.computeProjectionMatrixFOV(60, 600 / 540., .01, 1000.)

	def video_capture(self):
		"""
		This is very, very slow at the moment
		"""
		img_arr = self.p.getCameraImage(600, 540, self.viewMatrix, self.projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.

		plt.imshow(np_img)
		plt.pause(0.001)

	def parse_log(self, filename, verbose=True):

	  	f = open(filename, 'rb')
	  	print('Opened'),
	  	print(filename)

	  	keys = f.readline().decode('utf8').rstrip('\n').split(',')
	  	fmt = f.readline().decode('utf8').rstrip('\n')

	  	# The byte number of one record
	  	sz = struct.calcsize(fmt)
	  	# The type number of one record
	  	ncols = len(fmt)

	  	if verbose:
	  		print('Keys:'), 
	  		print(keys)
	  		print('Format:'),
	  		print(fmt)
	  		print('Size:'),
	  		print(sz)
	  		print('Columns:'),
	  		print(ncols)

	  	# Read data
	  	wholeFile = f.read()
	  	# split by alignment word
	  	chunks = wholeFile.split(b'\xaa\xbb')
	  	log = list()
	  	for chunk in chunks:
		    if len(chunk) == sz:
		      	values = struct.unpack(fmt, chunk)
		      	record = list()
		      	for i in range(ncols):
		        	record.append(values[i])
		      	log.append(record)

	  	return log

	def replay_log(self, log, delay=0.0005):

		for record in log:
			time_stamp = float(record[1])
			obj = record[2]
			pos = record[3: 6]
			orn = record[6: 10]
			self.p.resetBasePositionAndOrientation(obj, pos, orn)
			numJoints = self.p.getNumJoints(obj)
			for i in range(numJoints):
				jointInfo = self.p.getJointInfo(obj, i)
				qIndex = jointInfo[3]
				if qIndex > -1:
					self.p.resetJointState(obj, i, record[qIndex - 7 + 17])
			time.sleep(delay)

	def quit(self, fp): # logId
		if isinstance(fp, list):
			for Id in fp:
				self.p.stopStateLogging(Id)
		else:
			fp.close()
			
		self.p.resetSimulation()
		self.p.disconnect()

	def load_default_env(self):
		self.p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
		self.pr2_gripper = self.p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)

		# Setup the pr2_gripper
		jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
		for jointIndex in range(self.p.getNumJoints(self.pr2_gripper)):
			self.p.resetJointState(self.pr2_gripper, jointIndex,jointPositions[jointIndex])

		# Use -1 for the base, constrained within controller
		self.pr2_cid = self.p.createConstraint(self.pr2_gripper,-1,-1,-1, self.p.JOINT_FIXED,
			[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000])

		self.p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		self.p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)
		self.p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		self.p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
		shelf = self.p.loadSDF("kiva_shelf/model.sdf")[0]
		self.p.resetBasePositionAndOrientation(shelf, [-0.700000,-2.200000,1.204500],[0.000000,0.000000,0.000000,1.000000])
		self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		self.p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)
		self.p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		self.p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)


class KukaArmVR(BulletPhysicsVR):

	def __init__(self, pybullet, task):

		super().__init__(pybullet, task)

		# Set boundaries on kuka arm
		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
		self.JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
		self.REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
		self.KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]

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

		# if e[self.BUTTONS][32] & self.p.VR_BUTTON_WAS_RELEASED:
		# 	_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
		# 	self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)
		if controller_event[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
			
			# self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z_orig, force=5)		
			# self.ik_helper(kuka, targetPos, (0, 1, 0, 0))
			if fixed:
				self.ik_helper(kuka, targetPos, (0, 1, 0, 0))
			else:
				_, _, z = self.p.getEulerFromQuaternion(controller_orn)
				eef_orn = self.p.getQuaternionFromEuler([0, -math.pi, z])
				self.ik_helper(kuka, targetPos, eef_orn)

		# p.resetBasePositionAndOrientation(kuka_gripper, p.getBasePositionAndOrientation(kuka_gripper)[0], eef_orien)
		# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
		# if e[self.BUTTONS][32] & p.VR_BUTTON_WAS_TRIGGERED:
			# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)

	def disengage(self, kuka, controller_event):

		if controller_event[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
			for jointIndex in range(self.p.getNumJoints(kuka)):
				# self.p.resetJointState(kuka, jointIndex, self.REST_JOINT_POS[jointIndex])
				self.p.setJointMotorControl2(kuka, jointIndex, self.p.POSITION_CONTROL, 
					self.REST_JOINT_POS[jointIndex], 0)

	def ik_helper(self, arm_id, eef_pos, eef_orien, nullSpace=True):

		if nullSpace:
			joint_pos = self.p.calculateInverseKinematics(arm_id, 6, eef_pos, eef_orien, 
				lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, 
				jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE, jointDamping=self.JOINT_DAMP)
		else:
			joint_pos = self.p.calculateInverseKinematics(arm_id, 6, eef_pos, eef_orien)
		for i in range(len(joint_pos)):
			self.p.setJointMotorControl2(arm_id, i, self.p.POSITION_CONTROL, 
				targetPosition=joint_pos[i], targetVelocity=1, force=self.MAX_FORCE)

	def euc_dist(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist


