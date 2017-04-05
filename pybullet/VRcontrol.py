import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
import numpy as np
from model import *

class SingleKukaVR(KukaVR):
	"""
	An easier task without grasping
	"""
	def __init__(self, pybullet, task):

		super(SingleKukaVR, self).__init__(pybullet, task)
		self.kuka = -2
		
	def record(self, file, video=False):
		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)
		try:
			if video:
				# Does logging only need to be called once with SharedMemory? 
				bodylog = self.p.startStateLogging(self.p.STATE_LOGGING_VIDEO_MP4, 
					'../examples/pybullet/' + file + '.mp4')
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					'../examples/pybullet/generic.' + file)
				# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')

			logIds = [bodyLog]
			cId = None
			while True:

				events = self.p.getVREvents()
				for e in (events):
					# If the user think one task is completed, 
					# he/she will push the menu button
					eef_pos = self.p.getLinkState(self.kuka, 6)[0]

					if not cId:		
						# If detected contact points
						touch = self.p.getContactPoints(self.kuka)
				
						# Only attach when sticked around the eef center
						for contact_point in touch:
							if self.euc_dist(eef_pos, contact_point[5]) < 0.01 and contact_point[2] not in range(3):
								cId = self.p.createConstraint(self.kuka, 6, contact_point[2], -1, 
									self.p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
								
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_IS_DOWN:
						if cId:
							self.p.removeConstraint(cId)
						cId = None

					# Allows robot arm control by VR controllers
					sq_len = self.euc_dist(eef_pos, e[1])		
					if sq_len < self.THRESHOLD * self.THRESHOLD:
						# eef_pos = self.p.getBasePositionAndOrientation()
						target_plane_pos = (e[1][0], e[1][1], 1.23)
						curr_pos = self.p.getLinkState(self.kuka, 6)[0]
						target_point_pos = (curr_pos[0], curr_pos[1], 0.525)  # e[1][2]
						eef_orn = (0, 1, 0, 0)
						
						if e[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
							self.ik_helper(self.kuka, target_point_pos, eef_orn, fixed=True)
						else: 
							self.ik_helper(self.kuka, target_plane_pos, eef_orn, fixed=True)
					else:
						self.disengage(self.kuka, e)
	
					# Add user interaction for task completion
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
							# self.p.resetSimulation()
							# self.p.removeAllUserDebugItems()
						self.p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
						# Can add line for mark here
						# so that in saved csv file, we know when one task is complete		

		except KeyboardInterrupt:
			self.quit(logIds)

	def _setup_robot(self):
		# Only load a kuka arm, no need for gripper this time
		self.kuka = self.p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 
			1.400000, -0.200000, 0.600000,
			0.000000, 0.000000, 0.000000, 1.000000)
		self.reset_kuka(self.kuka)


class DoubleKukaVR(KukaVR):

	# This one try out the new loggingState method
	def __init__(self, pybullet, task):

		super(DoubleKukaVR, self).__init__(pybullet, task)
		self.kuka_arms = []
		self.kuka_grippers = []
		self.kuka_constraints = []

	def record(self, file, video=False):
		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)

		try:
			gripperMap = dict(zip(self.controllers, self.kuka_grippers))
			kukaMap = dict(zip(self.controllers, self.kuka_arms))
			
			if video:
				# Does logging only need to be called once with SharedMemory? 
				bodylog = self.p.startStateLogging(self.p.STATE_LOGGING_VIDEO_MP4, 
					'../examples/pybullet/' + file + '.mp4')
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					'../examples/pybullet/generic.' + file)
				# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')

			logIds = [bodyLog]

			while True:

				events = self.p.getVREvents()
				for e in (events):

					# If the user think one task is completed, 
					# he/she will push the menu button
					kuka_gripper = gripperMap[e[0]]
					kuka = kukaMap[e[0]]			

					#TODO: Add slider for the grippers
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.POSITION_CONTROL, 
								targetPosition=self.KUKA_GRIPPER_CLOZ_POS[i], force=50)

					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.POSITION_CONTROL, 
								targetPosition=self.KUKA_GRIPPER_REST_POS[i], force=50)		
					
					#TODO: Modify this
					sq_len = self.euc_dist(self.p.getLinkState(kuka, 6)[0], e[1])

					# Allows robot arm control by VR controllers
					if sq_len < self.THRESHOLD * self.THRESHOLD:

						current_x = self.p.getEulerFromQuaternion(self.p.getLinkState(kuka, 6)[1])[0]
						current_y = self.p.getEulerFromQuaternion(self.p.getLinkState(kuka, 5)[1])[1]
						ctrl_x, ctrl_y, _ = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
						# print (abs(current_x - ctrl_x), abs(current_y - ctrl_y))
						# if abs(current_y - ctrl_y) < math.pi / 4:

						self.engage(kuka, e, fixed=False)
					else:
						self.disengage(kuka, e)

					# Add user interaction for task completion
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
						# self.p.resetSimulation()
						# self.p.removeAllUserDebugItems()
						self.p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
						# Can add line for mark here
						# so that in saved csv file, we know when one task is complete		

		except KeyboardInterrupt:
			self.quit(logIds)

	def _setup_robot(self):
		pos = [0.3, -0.5]		# Original y-coord for the robot arms
		# Gripper ID to kuka arm ID
		for i in range(2):
			self.kuka_arms.append(self.p.loadURDF('kuka_iiwa/model_vr_limits.urdf', 1.4, pos[i], 0.6, 0, 0, 0, 1))
			self.kuka_grippers.append(self.p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')[0])
		
		# Setup initial conditions for both arms
		for kuka in self.kuka_arms:
			self.reset_kuka(kuka)

		# Setup initial conditions for both grippers
		for kuka_gripper in self.kuka_grippers:
			self.reset_kuka_gripper(kuka_gripper)
			
		# Setup constraints on kuka grippers
		for kuka, kuka_gripper in zip(self.kuka_arms, self.kuka_grippers):
			self.kuka_constraints.append(self.p.createConstraint(kuka,
				6, kuka_gripper, 0, self.p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0], parentFrameOrientation=[0, 0, 0, 1]))


class PR2GripperVR(BulletVR):

	def __init__(self, pybullet, task):

		super(PR2GripperVR, self).__init__(pybullet, task)
		self.pr2_gripper = 0
		self.pr2_cid = 0
		self.gripper_max_joint = 0.550569
		self.completed_task = {}
		self.obj_cnt = 0
		self.boxes = {}

	def create_scene(self):
		"""
		Basic scene needed for running tasks
		"""
		self.p.resetSimulation()
		self.p.setGravity(0, 0, -9.81)
		self.load_default_env()
		self.obj_cnt = self.p.getNumBodies()

		#TODO: think about extracting this bounding box out to avoid repeating code if 
		# this gripper works, abandon demoVR
		# Use loadArm=True/False for demoVR load_default_env 
		for obj in self.task:
			iD = self.p.loadURDF(*obj)
			self.p.addUserDebugText(str(iD - self.obj_cnt), 
				self.p.getBasePositionAndOrientation(iD)[0], textSize=8, lifeTime=0)
		#TODO: add labels
		self._load_boxes(numOfBoxes=9)

	def record(self, file, video=False):

		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)
		try:
			if video:
				# Does logging only need to be called once with SharedMemory? 
				bodylog = self.p.startStateLogging(self.p.STATE_LOGGING_VIDEO_MP4, 
					'../examples/pybullet/' + file + '.mp4')
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					'../examples/pybullet/generic.' + file)
				# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')

			logIds = [bodyLog]

			# pr2_release_pos = [ 0.550569, 0.000000, 0.549657, 0.000000 ]
			# pr2_trigger_pos = [0.04305865, 0, 0.04305865, 0]
			
			while True:

				events = self.p.getVREvents()
				for e in (events):

					# PR2 gripper follows VR controller				
					self.p.changeConstraint(self.pr2_cid, e[1], e[self.ORIENTATION], maxForce=500)	

					# if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
					# 	for i in range(self.p.getNumJoints(self.pr2_gripper)):
					# 		self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, 
					# 			targetPosition=pr2_trigger_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=50)

					# if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
					# 	for i in range(self.p.getNumJoints(self.pr2_gripper)):
					# 		self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, 
					# 			targetPosition=pr2_release_pos[i], targetVelocity=0, positionGain=0.05, velocityGain=1.0, force=50)

					# Setup gliders
					self.p.setJointMotorControl2(self.pr2_gripper, 0, self.p.POSITION_CONTROL, 
						targetPosition=self.gripper_max_joint * (1 - e[3]), force=5.0)
					self.p.setJointMotorControl2(self.pr2_gripper, 2, self.p.POSITION_CONTROL, 
						targetPosition=self.gripper_max_joint * (1 - e[3]), force=5.0)

					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
						self.p.addUserDebugText('One Item Inserted', (1.7, 0, 1), (255, 0, 0), 12, 10)

		except KeyboardInterrupt:
			self.quit(logIds)

	def _check_task(self):
		# Only check boundaries for objects in task
		for obj in range(self.obj_cnt, self.p.getNumBodies()):
			if obj not in self.completed_task:

				obj_pos = self.p.getBasePositionAndOrientation(obj)[0]
				bound = self.boxes[obj - self.obj_cnt]

				if self._fit_boundary(obj_pos, obj, bound):
					self.p.addUserDebugText('Finished', obj_pos, [1, 0, 0], lifeTime=5.)
					self._fit_routine(obj_pos, obj, bound)
					
	def _fit_routine(self, obj_pos, obj, boundary):
		self.complete_task[obj] = True
		# Change color
		for line, vertex in self.boxes[boundary]:
			self.p.removeUserDebugItem(line)
			self.p.addUserDebugLine(vertex[0], vertex[1], lineColorRGB=(0, 1, 0), lifeTime=0)
		# Hardcoded fact
		tableID = 14
		tablePosition = self.p.getBasePositionAndOrientation(tableID)[0]
		relPosition = [obj_pos[i] - tablePosition[i] for i in range(3)]
		# Add constraint
		self.p.createConstraint(tableID, 0, obj, 0, self.p.JOINT_POINT2POINT, [0, 0, 0], 
			relPosition, [0, 0, 0])

	def _fit_boundary(self, position, obj, boundary):

		table_top = [i[2] for i in self.p.getContactPoints(14)]	# hardcoded table
		return boundary[0][0] < position[0] < boundary[1][0] and boundary[0][1] < position[1] < boundary[1][1]\
			and obj in table_top
			
	def _load_boxes(self, startPos=(1.0, -0.7), numOfBoxes=3, size=0.07, interval=0.01, 
		height=0.63, color=(1, 0, 0)):
		"""
		Currently display the box shapes on the table surface
		"""
		for i in range(numOfBoxes):
			a_i_x = startPos[0] + i * (size + interval)
			a_i_y = startPos[1] 
			b_i_x = startPos[0] + i * (size + interval) + size
			b_i_y = startPos[1] + size
			self.boxes[i] = (((a_i_x, a_i_y), (b_i_x, b_i_y)))

		def construct_box(box_num):
			diag_a, diag_b = self.boxes[box_num]
			ax, ay = diag_a
			bx, by = diag_b
			v_a = (ax, ay, height)
			v_b = (bx, ay, height)
			v_c = (bx, by, height)
			v_d = (ax, by, height)
			a = self.p.addUserDebugLine(v_a, v_b, lineColorRGB=color, lifeTime=0)
			b = self.p.addUserDebugLine(v_b, v_c, lineColorRGB=color, lifeTime=0)
			c = self.p.addUserDebugLine(v_c, v_d, lineColorRGB=color, lifeTime=0)
			d = self.p.addUserDebugLine(v_d, v_a, lineColorRGB=color, lifeTime=0)
			# Label the box
			self.p.addUserDebugText(str(box_num), ((ax + bx) / 2., (ay + by) / 2., height), 
				textSize=8, lifeTime=0)
			# Keep track of the box region
			self.boxes[box_num] = (
								   (a, (v_a, v_b)), 
								   (b, (v_b, v_c)),
								   (c, (v_c, v_d)),
								   (d, (v_d, v_a))
								  )
		for k in self.boxes.keys():
			construct_box(k)


class DemoVR(BulletVR):

	# Still use current logging by myself to record grasp events
	def __init__(self, pybullet, task):

		super(DemoVR, self).__init__(pybullet, task)
		self.pr2_gripper = 2
		self.completed_task = {}
		self.obj_cnt = 0
		self.boxes = {}

	def create_scene(self, flag):
		"""
		Basic scene needed for running tasks
		"""
		load_status = -1
		while load_status < 0:
			if flag:
				load_status = self.p.connect(self.p.SHARED_MEMORY)
			else:
				load_status = self.p.connect(self.p.GUI)

		self.p.setGravity(0, 0, -9.81)

		if flag:
			self.obj_cnt = self.p.getNumBodies()
			for obj in self.task:
				iD = self.p.loadURDF(*obj)

				#TODO: better way to add labels for loaded objects
				# self.p.addUserDebugText(str(iD - self.obj_cnt), 
				# 	self.p.getBasePositionAndOrientation(iD)[0], textSize=8, lifeTime=0)

		self._load_boxes(numOfBoxes=9)

	def record(self, file, video=False):

		self.create_scene(1)
		try:
			if video:
				# Does logging only need to be called once with SharedMemory? 
				bodylog = self.p.startStateLogging(self.p.STATE_LOGGING_VIDEO_MP4, 
					'../examples/pybullet/' + file + '.mp4')
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					'../examples/pybullet/generic.' + file)
				# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')

			logIds = [bodyLog]
			while True:
				self._check_task()

				# wsgstates = [self.p.getJointState(7, i)[0] for i in range(self.p.getNumJoints(7))]
				
				# events = self.p.getVREvents()
				# for e in (events):

				# 	print('wsg', wsgstates)

				# 	if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
				# 		self.p.addUserDebugText('One Task Completed', (1.7, 0, 1), (255, 0, 0), 12, 10)

		except KeyboardInterrupt:
			self.quit(logIds)

	def replay(self, file, delay=1e-9):
		self.create_scene(0)
		self.p.setRealTimeSimulation(0)

		# Sorry, but must follow the same order of initialization as in compiled executable demo
		objects = [self.p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
		pr2_gripper = objects[0]

		jointPositions = [ 0.550569, 0.000000, 0.549657, 0.000000 ]
		for jointIndex in range (self.p.getNumJoints(pr2_gripper)):
			self.p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])

		objects = [self.p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
		kuka = objects[0]
		jointPositions = [ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
		for jointIndex in range (self.p.getNumJoints(kuka)):
			self.p.resetJointState(kuka,jointIndex,jointPositions[jointIndex])
			self.p.setJointMotorControl2(kuka,jointIndex,self.p.POSITION_CONTROL,jointPositions[jointIndex],0)

		objects = [self.p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.800000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.900000,0.000000,0.000000,0.000000,1.000000)]

		objects = self.p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
		kuka_gripper = objects[0]
		self.p.resetBasePositionAndOrientation(kuka_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
		jointPositions = [ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
		for jointIndex in range (self.p.getNumJoints(kuka_gripper)):
			self.p.resetJointState(kuka_gripper,jointIndex,jointPositions[jointIndex])
			self.p.setJointMotorControl2(kuka_gripper,jointIndex,self.p.POSITION_CONTROL,jointPositions[jointIndex],0)
		kuka_cid = self.p.createConstraint(kuka, 6, kuka_gripper, 0, self.p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0], childFrameOrientation=[0, 0, 0, 1])

		objects = [self.p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
		objects = [self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
		objects = [self.p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
		objects = [self.p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
		objects = [self.p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
		objects = [self.p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
		objects = self.p.loadSDF("kiva_shelf/model.sdf")
		self.p.resetBasePositionAndOrientation(objects[0],[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
		objects = [self.p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)]
		objects = [self.p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
		objects = [self.p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
		ob = objects[0]
		jointPositions = [ 0.000000 ]
		for jointIndex in range (self.p.getNumJoints(ob)):
			self.p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

		objects = [self.p.loadURDF("husky/husky.urdf", 2.000000,-5.000000,1.000000,0.000000,0.000000,0.000000,1.000000)]
		ob = objects[0]
		jointPositions = [ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 ]
		for jointIndex in range (self.p.getNumJoints(ob)):
			self.p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

		for obj in self.task:
			self.p.loadURDF(*obj)

		# Setup the camera 
		self.p.resetDebugVisualizerCamera(cameraDistance=self.FOCAL_LENGTH, 
			cameraYaw=self.YAW, cameraPitch=self.PITCH, 
			cameraTargetPosition=self.FOCAL_POINT)

		log = self.parse_log('generic.' + file, verbose=False)

		self.replay_log(log, delay=delay)
		self.quit([])

	def _check_task(self):
		# Only check boundaries for objects in task
		for obj in range(self.obj_cnt, self.p.getNumBodies()):
			if obj not in self.completed_task:

				obj_pos = self.p.getBasePositionAndOrientation(obj)[0]
				bound = self.boxes[obj - self.obj_cnt]

				if self._fit_boundary(obj_pos, obj, bound):
					self.p.addUserDebugText('Finished', obj_pos, [1, 0, 0], lifeTime=5.)
					self._fit_routine(obj_pos, obj, bound)
					
	def _fit_routine(self, obj_pos, obj, boundary):
		# self.complete_task[obj] = True
		# Change color
		for line, vertex in boundary:
			self.p.removeUserDebugItem(line)
			self.p.addUserDebugLine(vertex[0], vertex[1], lineColorRGB=(0, 1, 0), lifeTime=0)
		# Hardcoded fact
		tableID = 14
		tablePosition = self.p.getBasePositionAndOrientation(tableID)[0]
		relPosition = [obj_pos[i] - tablePosition[i] for i in range(3)]
		# Add constraint
		# print(tablePosition)
		self.p.createConstraint(-1, -1, obj, -1, self.p.JOINT_FIXED, [0, 0, 0], 
			 obj_pos, [0, 0, 0])

	def _fit_boundary(self, position, obj, boundary):

		table_top = [i[2] for i in self.p.getContactPoints(14)]	# hardcoded table
		# print(position, 'pos')
		# print (boundary, 'bod')
		return boundary[0][1][0][0] < position[0] < boundary[0][1][1][0] and \
			boundary[0][1][1][1] < position[1] < boundary[2][1][1][1] and obj in table_top
		# all([(boundary[0][i] - boundary[1][i] / 2) <= position[i]\
		# 	<= (boundary[0][i] + boundary[1][i] / 2)  for i in range(2)])
			
	def _load_boxes(self, startPos=(0.6, -0.7), numOfBoxes=3, size=0.07, interval=0.01, 
		height=0.63, color=(1, 0, 0)):
		"""
		Currently display the box shapes on the table surface
		"""
		for i in range(numOfBoxes):
			a_i_x = startPos[0] + i * (size + interval)
			a_i_y = startPos[1] 
			b_i_x = startPos[0] + i * (size + interval) + size
			b_i_y = startPos[1] + size
			self.boxes[i] = (((a_i_x, a_i_y), (b_i_x, b_i_y)))

		def construct_box(box_num):
			diag_a, diag_b = self.boxes[box_num]
			ax, ay = diag_a
			bx, by = diag_b
			v_a = (ax, ay, height)
			v_b = (bx, ay, height)
			v_c = (bx, by, height)
			v_d = (ax, by, height)
			a = self.p.addUserDebugLine(v_a, v_b, lineColorRGB=color, lifeTime=0)
			b = self.p.addUserDebugLine(v_b, v_c, lineColorRGB=color, lifeTime=0)
			c = self.p.addUserDebugLine(v_c, v_d, lineColorRGB=color, lifeTime=0)
			d = self.p.addUserDebugLine(v_d, v_a, lineColorRGB=color, lifeTime=0)

			#TODO: better way to Label the box
			# self.p.addUserDebugText(str(box_num), ((ax + bx) / 2., (ay + by) / 2., height), 
			# 	textSize=8, lifeTime=0)

			# Keep track of the box region
			self.boxes[box_num] = (
								   (a, (v_a, v_b)), 
								   (b, (v_b, v_c)),
								   (c, (v_c, v_d)),
								   (d, (v_d, v_a))
								  )
		for k in self.boxes.keys():
			construct_box(k)



