import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
import numpy as np

class BulletPhysicsVR(object):

	def __init__(self, pybullet, task=0, hand=False):
		# Create tasks as first step
		self.tasks = self._init_task()
		self.task = task
		self.p = pybullet

		self.VR_HAND_ID = None
		self.BUTTONS = 6
		self.ORIENTATION = 2
		self.controllers = [3, 4]
		self.kuka_arms = []
		self.kuka_grippers = []
		self.kuka_constraints = []

		# Default settings for camera
		self.FOCAL_POINT = (0., 0., 0.)
		self.YAW = 35.
		self.PITCH = 50.
		self.ROLL = 0.
		self.FOCAL_LENGTH = 5.
		self.UP_AX_IDX = 2

		self.THRESHOLD = 1.15
		self.MAX_FORCE = 500
		self.ROBOT_MAP = {}
		self.viewMatrix = None
		self.projectionMatrix = None
		self.hand = hand
		self.ROBOT_MAP = {}

		self.LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
		self.UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
		self.JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
		self.REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]

	def _load_task(self, flag):
		"""
		Load task for both recording and replay
		"""
		try:
			if flag:
				self.p.connect(self.p.GUI)
			else:
				self.p.connect(self.p.SHARED_MEMORY)
			self.p.setInternalSimFlags(0)
			if flag:
				self.p.setRealTimeSimulation(0)
			else:
				self.p.setRealTimeSimulation(1)
			# Use GUI and turn off simulation for replay
			# In order to generate deterministic paths
			# convenient for video recording
		
		except self.p.error:
			return 0
		self._init_scene()
		for t in self.tasks[self.task]:
			self.p.loadURDF(*t)
		return 1

	def record(self, file):
		load_status = 0
		while load_status == 0:
			self.p.connect(self.p.SHARED_MEMORY)
			load_status = self._load_task(0)
		try:
			f = open(file, 'w', newline='')
			writer = csv.writer(f)
			prev_time = time.time()

			gripperMap = dict(zip(self.controllers, self.kuka_grippers))
			kukaMap = dict(zip(self.controllers, self.kuka_arms))

			while True:
				events = self.p.getVREvents()

				for e in (events):
					# If the user think one task is completed, 
					# he/she will push the menu button
					# controller_pos, controller_orien = e
					kuka_gripper = gripperMap[e[0]]
					kuka = kukaMap[e[0]]
					# _, _, z = self.p.getEulerFromQuaternion(e[2])
					# self.p.setJointMotorControl2(1, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)

					# Add sliders for gripper joints
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_TRIGGERED:
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=5, force=50)
						row = [-2, kuka_gripper]
						writer.writerow(row)
					if e[self.BUTTONS][33] & self.p.VR_BUTTON_WAS_RELEASED:	
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=-5, force=50)
						row = [-3, kuka_gripper]
						writer.writerow(row)
					sq_len = self._euc_dist(self.p.getBasePositionAndOrientation(kuka_gripper)[0], e[1])
					
					if sq_len < self.THRESHOLD * self.THRESHOLD:
						# time = 0.0
						# time += 0.01
						# targetPos = (0.4 - 0.4 * math.cos(time), 0, 0.8 + 0.4 * math.cos(time))
						# eef_pos = (e[1][0] + targetPos[0], e[1][1] + targetPos[1], e[1][2] + targetPos[2])
						targetPos = e[1]
						
						x, y, z_orig = self.p.getEulerFromQuaternion((0, 1, 0, 0))
						_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
						# # print(x, y, z)
						# eef_orien = p.getBasePositionAndOrientation(kuka_gripper)[1]
						eef_orien = self.p.getQuaternionFromEuler([x, y, z])
						# print(eef_orien)

						if e[self.BUTTONS][32] & self.p.VR_BUTTON_WAS_RELEASED:
						
							_, _, z = self.p.getEulerFromQuaternion(e[self.ORIENTATION])
							self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z, force=5)
						
						if e[self.BUTTONS][32] & self.p.VR_BUTTON_IS_DOWN:
								
							self.p.setJointMotorControl2(kuka, 6, self.p.POSITION_CONTROL, targetPosition=z_orig, force=5)
							joint_pos = self.p.calculateInverseKinematics(kuka, 6, targetPos, (0, 1, 0, 0), 
								lowerLimits=self.LOWER_LIMITS, upperLimits=self.UPPER_LIMITS, jointRanges=self.JOINT_RANGE, restPoses=self.REST_POSE)
							for i in range(len(joint_pos)):
								self.p.setJointMotorControl2(kuka, i, self.p.POSITION_CONTROL, targetPosition=joint_pos[i], force=500)

			
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


				# Saving events routine
				if events and events[0][5] > 0:	# Only take record when moving events happen
				# Removing this line for video generation may sync the process?
					curr_time = time.time()
					time_elapse = curr_time - prev_time
					prev_time = curr_time
					for o_id in range(self.p.getNumBodies()):
						# Track objects
						# if o_id not in self.grippers:
						row = [(o_id)] + list(self.p.getBasePositionAndOrientation(o_id)[0]) + list(self.p.getBasePositionAndOrientation(o_id)[1])
						writer.writerow(row)
					# Write extra vr_hand now
					# if self.hand:
					# 	hand = [self.VR_HAND_ID] + list(self.p.getBasePositionAndOrientation(self.VR_HAND_ID)[0]) + list(self.p.getBasePositionAndOrientation(self.VR_HAND_ID)[1])
					# 	writer.writerow(hand)
						# else:
						# 	gripper_info = ['g'] + [o_id] + list(self.p.getBasePositionAndOrientation(o_id)[0]) + \
						# 		list(self.p.getBasePositionAndOrientation(o_id)[1])
						# 	writer.writerow(gripper_info)
					
					delay = [-1, time_elapse]
					writer.writerow(delay)

		except KeyboardInterrupt:
			self._exit_routine(f)

	def replay(self, file, saveVideo=0):
		load_status = 0
		while load_status == 0:
			load_status = self._load_task(1)
		# Setup the camera 
		self.p.setCameraViewPoint(self.FOCAL_POINT[0], self.FOCAL_POINT[1], self.FOCAL_POINT[2], 
			self.PITCH, self.YAW, self.FOCAL_LENGTH)

		f = open(file, 'r')
		reader = csv.reader(f)
		delay = 0
		for row in reader:
			if int(row[0]) == -1:
				delay = float(row[1]) / 15
			else:
				time.sleep(delay)
				# Keep the simulation synced

				if int(row[0]) not in self.kuka_grippers:
					if int(row[0]) == -2:
						kuka_gripper = int(row[1])
						# self.p.setRealTimeSimulation(1)
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=5, force=50)
						# self.p.setRealTimeSimulation(0)
					elif int(row[0]) == -3:
						kuka_gripper = int(row[1])
						# self.p.setRealTimeSimulation(1)
						for i in range(self.p.getNumJoints(kuka_gripper)):
							self.p.setJointMotorControl2(kuka_gripper, i, self.p.VELOCITY_CONTROL, targetVelocity=-5, force=50)
						# self.p.setRealTimeSimulation(0)
					else:
						self.p.resetBasePositionAndOrientation(int(row[0]), (float(row[1]), float(row[2]), 
							float(row[3])), (float(row[4]), float(row[5]), float(row[6]), float(row[7])))

				else:
					# Assert load order for the plane, robot, and gripper are the same
					eef_pos = (float(row[1]), float(row[2]), float(row[3]))
					eef_orien = (float(row[4]), float(row[5]), float(row[6]), float(row[7]))
					joint_pos = self.p.calculateInverseKinematics(self.ROBOT_MAP[int(row[0])], 6, 
						eef_pos, eef_orien)
					self.p.resetBasePositionAndOrientation(self.ROBOT_MAP[int(row[0])] + 1, 
						eef_pos, eef_orien)
					for i in range(len(joint_pos)):
						self.p.resetJointState(self.ROBOT_MAP[int(row[0])], i, joint_pos[i])
			
				if saveVideo:
					self.video_capture()
		self._exit_routine(f)


	def video_capture(self):
		"""
		This is very, very slow at the moment
		"""
		img_arr = self.p.getCameraImage(600, 540, self.viewMatrix, self.projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.

		plt.imshow(np_img)
		plt.pause(0.001)

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

	def _euc_dist(self, posA, posB):
		dist = 0.
		for i in range(len(posA)):
			dist += (posA[i] - posB[i]) ** 2
		return dist

	def _ik_helper(self, arm_id, eef_pos, eef_orien):

		joint_pos = self.p.calculateInverseKinematics(arm_id, 6, eef_pos, eef_orien)
		for i in range(len(joint_pos)):
			self.p.setJointMotorControl2(arm_id, i, self.p.POSITION_CONTROL, targetPosition=joint_pos[i], force=self.MAX_FORCE)


	def _init_scene(self):
		"""
		Basic scene needed for running tasks
		"""
		self.p.resetSimulation()
		self.p.setGravity(0, 0, -9.81)
		if self.hand:
			self.VR_HAND_ID = self.p.loadMJCF("MPL/mpl2.xml")[0]
		# self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		self.p.loadURDF("plane.urdf",0,0,0,0,0,0,1)
		self.p.loadURDF("table/table.urdf", -1.0,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		self._setup_robot()

	def _setup_robot(self):
		pos = [0.3, -0.5]		# Original y-coord for the robot arms
		for i in range(2):		# Setup two arms
			self.kuka_arms.append(self.p.loadURDF('kuka_iiwa/model_vr_limits.urdf', -0.1, pos[i], 0.6, 0, 0, 0, 1))
			self.kuka_grippers.append(self.p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')[0])

		kuka_jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
		
		# Setup initial conditions for both arms
		for kuka in self.kuka_arms:
			for jointIndex in range(self.p.getNumJoints(kuka)):
				self.p.resetJointState(kuka, jointIndex, kuka_jointPositions[jointIndex])
				self.p.setJointMotorControl2(kuka,jointIndex, self.p.POSITION_CONTROL, 
					kuka_jointPositions[jointIndex], 0)
				# pos = [0.28, -0.95]

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
			self.ROBOT_MAP[kuka_gripper] = kuka
			# Gripper ID to kuka arm ID

	def _init_task(self):
		repo = {}
		# Indicate the indices of the objects that need to be tracked in the first entry
		repo[0] = [("cvgl/pole.urdf",-0.80000,0.100000,0.699990,0.00000,0.0,0.00000,1), 
			("cvgl/pole.urdf",-0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1),
			("cvgl/pole.urdf",-0.80000,-0.500000,0.699990,0.00000,0.0,0.00000,1),
			("cvgl/torus_0.urdf",-0.8,0.1,0.69999,1,0,0,1),
			("cvgl/torus_1.urdf",-0.8,0.1,0.74999,1,0,0,1),
			("cvgl/torus_2.urdf",-0.8,0.1,0.79999,1,0,0,1),
			("cvgl/torus_3.urdf",-0.8,0.1,0.84999,1,0,0,1),
			("cvgl/torus_4.urdf",-0.8,0.1,0.89999,0,0,0,1)]

		repo[1] = [("sphere_small.urdf",-0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1),
				("sphere_small.urdf",-0.76000,-0.1400000,0.729990,0.000000,0.0,0.00000,1),
				("sphere_small.urdf",-0.83000,-0.520000,0.699990,0.000000,0.0,0.00000,1),
				("tray/tray_textured2.urdf", -0.56, -0.11, 0.6, 0, 0, 0, 1)]

		# repo[2] = [(, ), ("", )
		# 		   (, ), ("", )
		# 		   (, ), ("", )]

		return repo

	def _exit_routine(self, fp):
		fp.close()
		self.p.resetSimulation()
		self.p.disconnect()

# def render_camera_port(focus_pt, focal_len, yaw, pitch, roll, upAxisIndex=2):
# 	viewMatrix = p.computeViewMatrixFromYawPitchRoll(focus_pt, focal_len, yaw, pitch, roll, upAxisIndex)
# 	projectionMatrix = p.computeProjectionMatrixFOV(60, 320/240., 0.01, 1000.)
# 	img_arr = p.getCameraImage(320, 240, viewMatrix, projectionMatrix)
# 	np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.
# 	fig = plt.figure()
# 	ax = fig.add_subplot(111)
# 	im = ax.imshow(np_img)
	
# 	def next_camera_frame():
# 		return p.getCameraImage(320, 240, viewMatrix, projectionMatrix)

# 	return animation.FuncAnimation(fig, next_camera_frame, interval=75) # draw/75ms


# obj = create_task_scene()
# ani = render_camera_port(FOCAL_POINT, FOCAL_LENGTH, YAW, PITCH, ROLL, UP_AX_IDX)
# # writer = animation.writers['ffmpeg'](fps=1000/75)
