import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
import numpy as np
from model import *

class KukaSingleArmVR(KukaArmVR):
	"""
	An easier task without grasping
	"""
	def __init__(self, pybullet, task):

		super().__init__(pybullet, task)
		self.kuka = -2
		
	def record(self, file):
		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)
		try:
			
			# Record everything
			bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
				'../examples/pybullet/generic.' + file)

			# # May not needed anymore; benchmark all events during replay is also possible
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
								cId = self.p.createConstraint(self.kuka, 6, contact_point[2], -1, self.p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
								
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_IS_DOWN:
						if cId:
							self.p.removeConstraint(cId)
						cId = None

					sq_len = self.euc_dist(eef_pos, e[1])

					# Allows robot arm control by VR controllers
					if sq_len < self.THRESHOLD * self.THRESHOLD:
						
						# eef_pos = self.p.getBasePositionAndOrientation()
						target_plane_pos = (e[1][0], e[1][1], 1.33)

						curr_pos = self.p.getLinkState(self.kuka, 6)[0]
						target_point_pos = (curr_pos[0], curr_pos[1], e[1][2])
						eef_orn = (0, 1, 0, 0)

						# if e[self.BUTTONS][32] & self.p.VR_BUTTON_WAS_RELEASED:
						# 	_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
						# 	self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)
						
						if e[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
							
							# self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z_orig, force=5)		
							# self.ik_helper(kuka, targetPos, (0, 1, 0, 0))
							self.ik_helper(self.kuka, target_point_pos, eef_orn)
						else: 
							self.ik_helper(self.kuka, target_plane_pos, eef_orn)

					else:
						jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
						for jointIndex in range(self.p.getNumJoints(self.kuka)):
							self.p.setJointMotorControl2(self.kuka,jointIndex, self.p.POSITION_CONTROL, jointPositions[jointIndex], 1)

					# Add user interaction for task completion
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
							# self.p.resetSimulation()
							# self.p.removeAllUserDebugItems()
						self.p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
						# Can add line for mark here
						# so that in saved csv file, we know when one task is complete		

		except KeyboardInterrupt:
			self.quit(logIds)

	def replay(self, file, saveVideo=0):
		
		load_status = 0
		while load_status == 0:
			load_status = self.setup(1)
		# Setup the camera 
		self.p.resetDebugVisualizerCamera(self.FOCAL_LENGTH, self.YAW, 
			self.PITCH, self.FOCAL_POINT)

		log = self.parse_log('generic.' + file, verbose=True)
		self.replay_log(log)

		# 		if saveVideo:
		# 			self.video_capture()
		self.quit([])


	def _setup_robot(self):
		# Only load a kuka arm, no need for gripper this time
		self.kuka = self.p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 
			1.400000, -0.200000, 0.600000,
			0.000000, 0.000000, 0.000000, 1.000000)
		jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
		for jointIndex in range(self.p.getNumJoints(self.kuka)):
			self.p.resetJointState(self.kuka, jointIndex, jointPositions[jointIndex])
			self.p.setJointMotorControl2(self.kuka, jointIndex, self.p.POSITION_CONTROL, 
				jointPositions[jointIndex], 0)



class KukaDoubleArmVR(KukaArmVR):

	# This one try out the new loggingState method
	def __init__(self, pybullet, task):

		super().__init__(pybullet, task)
		self.kuka_arms = []
		self.kuka_grippers = []
		self.kuka_constraints = []

	def record(self, file, saveVideo=0):
		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)
		try:

			gripperMap = dict(zip(self.controllers, self.kuka_grippers))
			kukaMap = dict(zip(self.controllers, self.kuka_arms))
			
			# Record everything
			bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
				'../examples/pybullet/generic.' + file)

			# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
			# 	file + '_ctrl')
			logIds = [bodyLog]

			while True:
				events = self.p.getVREvents()

				for e in (events):
					# # If the user think one task is completed, 
					# # he/she will push the menu button
					# # controller_pos, controller_orien = e
					kuka_gripper = gripperMap[e[0]]
					kuka = kukaMap[e[0]]			

					# Add sliders for gripper joints
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=5, force=50)
						# row = [-2, kuka_gripper]
						# writer.writerow(row)
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=-5, force=50)
						# row = [-3, kuka_gripper]
						# writer.writerow(row)
					sq_len = self.euc_dist(self.p.getBasePositionAndOrientation(kuka_gripper)[0], e[1])
					
					# Allows robot arm control by VR controllers
					if sq_len < self.THRESHOLD * self.THRESHOLD:
	
						targetPos = e[1]
						
						# x, y, z_orig = self.p.getEulerFromQuaternion((0, 1, 0, 0))
						_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
						eef_orn = self.p.getQuaternionFromEuler([0, -math.pi, z])

						# if e[self.BUTTONS][32] & self.p.VR_BUTTON_WAS_RELEASED:
						# 	_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
						# 	self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)
						
						if e[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
							
							# self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z_orig, force=5)		
							# self.ik_helper(kuka, targetPos, (0, 1, 0, 0))
							
							self.ik_helper(kuka, targetPos, eef_orn)

						# p.resetBasePositionAndOrientation(kuka_gripper, p.getBasePositionAndOrientation(kuka_gripper)[0], eef_orien)

						# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
						# if e[self.BUTTONS][32] & p.VR_BUTTON_WAS_TRIGGERED:

							# p.setJointMotorControl2(kuka, 6, p.POSITION_CONTROL, targetPosition=z, force=5)
					else:
						jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
						for jointIndex in range(self.p.getNumJoints(kuka)):
							self.p.setJointMotorControl2(kuka,jointIndex, self.p.POSITION_CONTROL, jointPositions[jointIndex], 1)

								# self.p.setJointMotorControl2(kukaMap[e[0]], 6, self.p.POSITION_CONTROL, targetPosition=math.pi, force=50)

					# Add user interaction for task completion
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
							# self.p.resetSimulation()
							# self.p.removeAllUserDebugItems()
						self.p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
						# Can add line for mark here
						# so that in saved csv file, we know when one task is complete		

		except KeyboardInterrupt:
			self.quit(logIds)

	def replay(self, file, saveVideo=0):
		load_status = 0
		while load_status == 0:
			load_status = self.setup(1)
		# Setup the camera 
		self.p.resetDebugVisualizerCamera(self.FOCAL_LENGTH, self.YAW, 
			self.PITCH, self.FOCAL_POINT)

		log = self.parse_log('generic.' + file, verbose=True)

		self.replay_log(log, delay=0.00045)

		# 		if saveVideo:
		# 			self.video_capture()
		self.quit([])

	def _setup_robot(self):
		pos = [0.3, -0.5]		# Original y-coord for the robot arms
		for i in range(2):		# Setup two arms
			self.kuka_arms.append(self.p.loadURDF('kuka_iiwa/model_vr_limits.urdf', 1.4, pos[i], 0.6, 0, 0, 0, 1))
			self.kuka_grippers.append(self.p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')[0])

		kuka_jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
		
		# Setup initial conditions for both arms
		for kuka in self.kuka_arms:
			for jointIndex in range(self.p.getNumJoints(kuka)):
				self.p.resetJointState(kuka, jointIndex, kuka_jointPositions[jointIndex])
				self.p.setJointMotorControl2(kuka,jointIndex, self.p.POSITION_CONTROL, 
					kuka_jointPositions[jointIndex], 0)
				# pos = [0.28, -0. 95]

		# Setup initial conditions for both grippers
		for kuka_gripper in self.kuka_grippers:
			self.p.resetBasePositionAndOrientation(kuka_gripper,
				[0.923103,-0.200000,1.250036],
				[-0.000000,0.964531,-0.000002,-0.263970])
			kuka_gripper_jointPositions = [0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000]
			for jointIndex in range(self.p.getNumJoints(kuka_gripper)):
				self.p.resetJointState(kuka_gripper, jointIndex, kuka_gripper_jointPositions[jointIndex])
				self.p.setJointMotorControl2(kuka_gripper, jointIndex, 
					self.p.POSITION_CONTROL, kuka_gripper_jointPositions[jointIndex], 0)

		# Setup constraints on kuka grippers
		for kuka, kuka_gripper in zip(self.kuka_arms, self.kuka_grippers):
			self.kuka_constraints.append(self.p.createConstraint(kuka,
				6, kuka_gripper, 0, self.p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0]))
			# Gripper ID to kuka arm ID


class PR2GripperVR(BulletPhysicsVR):

	def __init__(self, pybullet, task):

		super().__init__(pybullet, task)
		self.pr2_gripper = 0
		self.pr2_cid = 0

	def create_scene(self):
		"""
		Basic scene needed for running tasks
		"""
		self.p.resetSimulation()
		self.p.setGravity(0, 0, -9.81)

		self.p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
		# self.p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
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
		# self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		# self.p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)
		# self.p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)
		# self.p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)
		# self.p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)
		shelf = self.p.loadSDF("kiva_shelf/model.sdf")[0]
		self.p.resetBasePositionAndOrientation(shelf, [-0.700000,-2.200000,1.204500],[0.000000,0.000000,0.000000,1.000000])
		# self.p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		# self.p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)
		# self.p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)
		# self.p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)

	def record(self, file):

		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self.setup(0)
		try:
			f = open(file, 'w', newline='')
			writer = csv.writer(f)
			prev_time = time.time()
			while True:
				events = self.p.getVREvents()

				for e in (events):

					# PR2 gripper follows VR controller				
					self.p.changeConstraint(self.pr2_cid, e[1], e[self.ORIENTATION], maxForce=100000)	

					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
						for i in range(self.p.getNumJoints(self.pr2_gripper)):
							self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, targetPosition=0, force=50)
						row = [-2]
						writer.writerow(row)
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
						for i in range(self.p.getNumJoints(self.pr2_gripper)):
							self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, targetPosition=1, force=50)
						row = [-3]
						writer.writerow(row)
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
						self.p.addUserDebugText('One Item Inserted', (1.7, 0, 1), (255, 0, 0), 12, 10)
				
				# Saving events routine
				if events and events[0][5] > 0:	# Only take record when moving events happen
				# Removing this line for video generation may sync the process?
					curr_time = time.time()
					time_elapse = curr_time - prev_time
					prev_time = curr_time
					for o_id in range(self.p.getNumBodies()):
						# Track objects
						row = [(o_id)] + list(self.p.getBasePositionAndOrientation(o_id)[0]) + list(self.p.getBasePositionAndOrientation(o_id)[1])
						writer.writerow(row)

					delay = [-1, time_elapse]
					writer.writerow(delay)

		except KeyboardInterrupt:
			self.quit(f)

	def replay(self, file, saveVideo=0):

		load_status = 0
		while load_status == 0:
			load_status = self.setup(1)
		# Setup the camera 
		self.p.resetDebugVisualizerCamera(self.FOCAL_LENGTH, self.YAW, 
			self.PITCH, self.FOCAL_POINT)

		f = open(file, 'r')
		reader = csv.reader(f)
		delay = 0
		for row in reader:
			obj_id = int(row[0])
			if obj_id == -1:
				delay = float(row[1]) / 50
			else:
				time.sleep(delay)
				# Keep the simulation synced
				if obj_id == -2:
					for i in range(self.p.getNumJoints(self.pr2_gripper)):
						self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, targetPosition=0, force=50)
				elif obj_id == -3:
					for i in range(self.p.getNumJoints(self.pr2_gripper)):
						self.p.setJointMotorControl2(self.pr2_gripper, i, self.p.POSITION_CONTROL, targetPosition=1, force=50)
				else:
					self.p.resetBasePositionAndOrientation(obj_id, (float(row[1]), float(row[2]), 
						float(row[3])), (float(row[4]), float(row[5]), float(row[6]), float(row[7])))

				if saveVideo:
					self.video_capture()
		self.quit(f)


class DemoVR(BulletPhysicsVR):

	# Still use current logging by myself to record grasp events
	def __init__(self, pybullet, task):

		super().__init__(pybullet, task)
		self.pr2_gripper = 2
		self.obj_cnt = 0
		# self.tracking_obj = None

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
		# self.p.resetSimulation()
		# Comment out the reset simulation to provide entire control and access to obj info...
		self.p.setGravity(0, 0, -9.81)
		self.obj_cnt = 25
		if flag:
			for obj in self.task:
				self.p.loadURDF(*obj)
		# self.tracking_obj = range(self.obj_cnt, self.p.getNumBodies())

	def record(self, file):

		self.create_scene(1)
		try:
			f = open(file, 'w', newline='')
			writer = csv.writer(f)
			prev_time = time.time()
			while True:
				events = self.p.getVREvents()

				for e in (events):
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
						self.p.addUserDebugText('One Task Completed', (1.7, 0, 1), (255, 0, 0), 12, 10)

					# Mark the grasp
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
						row = [-3]
						writer.writerow(row)
					# Mark the release
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
						row = [-4]
						writer.writerow(row)

					# _, _, z = self.p.getEulerFromQuaternion(e[2])
					# self.p.setJointMotorControl2(3, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)
				
				# Saving events routine
				if events and events[0][5] > 0:	# Only take record when moving events happen
				# Removing this line for video generation may sync the process?
					curr_time = time.time()
					time_elapse = curr_time - prev_time
					prev_time = curr_time

					# Need to track everything
					# for o_id in self.tracking_obj:
					# for o_id in range(self.p.getNumBodies() + self.obj_cnt):
					for o_id in range(self.p.getNumBodies()):
						# This suggests the robot arm is chosen
						if self.p.getNumJoints(o_id) == 7:
							jointStates = [self.p.getJointState(o_id, i)[0] for i in range(self.p.getNumJoints(o_id))]
							row = [(-2)] + jointStates

						# # This suggests the pr2 gripper
						# elif o_id == 2:

						else:
							row = [(o_id)] + list(self.p.getBasePositionAndOrientation(o_id)[0]) + list(self.p.getBasePositionAndOrientation(o_id)[1])
						writer.writerow(row)

					gripper_info = [self.pr2_gripper] + list(self.p.getBasePositionAndOrientation(self.pr2_gripper)[0])\
						 + list(self.p.getBasePositionAndOrientation(self.pr2_gripper)[1])
					delay = [-1, time_elapse]
					writer.writerow(gripper_info)
					writer.writerow(delay)

		except KeyboardInterrupt:
			self.quit(f)

	def replay(self, file, saveVideo=0):
		self.create_scene(0)
		self.p.setRealTimeSimulation(0)
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
		ob = objects[0]
		self.p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
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
		self.p.resetDebugVisualizerCamera(self.FOCAL_LENGTH, self.YAW, 
			self.PITCH, self.FOCAL_POINT)

		f = open(file, 'r')
		reader = csv.reader(f)
		delay = 0
		for row in reader:
			obj_id = int(row[0])
			if obj_id == -1:
				delay = float(row[1]) / 1000
			else:
				time.sleep(delay)
				# Keep the simulation synced
				if obj_id == -2:
					for i in range(self.p.getNumJoints(kuka)):
						self.p.resetJointState(kuka, i, float(row[i + 1]))
				else:
					if obj_id == -3 or obj_id == -4:
						continue
					self.p.resetBasePositionAndOrientation(obj_id, (float(row[1]), float(row[2]), 
						float(row[3])), (float(row[4]), float(row[5]), float(row[6]), float(row[7])))
			
				if saveVideo:
					self.video_capture()
		self.quit(f)






			



