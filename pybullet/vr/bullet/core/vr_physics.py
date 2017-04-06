import struct
import time, os
import numpy as np
from os.path import join as pjoin

class BulletVR(object):

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

		self.VIDEO_DIR = pjoin(os.getcwd(), 'data', 'video')
		self.RECORD_LOG_DIR = pjoin(os.getcwd(), 'data', 'record')

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

	def record(self, file, video=False):
		raise NotImplementedError("Each VR Setup must re-implement this method.")

	def replay(self, file, delay=0.0001):
		load_status = 0
		while load_status == 0:
			load_status = self.setup(1)
		# Setup the camera 
		self.p.resetDebugVisualizerCamera(cameraDistance=self.FOCAL_LENGTH, 
			cameraYaw=self.YAW, cameraPitch=self.PITCH, 
			cameraTargetPosition=self.FOCAL_POINT)
		log = self.parse_log(pjoin(self.RECORD_LOG_DIR, 'generic.' + file), verbose=True)
		self.replay_log(log, delay=delay)
		self.quit([])

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

