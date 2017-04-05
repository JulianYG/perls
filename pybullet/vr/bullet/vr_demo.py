from os.path import join as pjoin
from core.vr_physics import BulletVR


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
					pjoin(self.VIDEO_DIR, file + '.mp4'))
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file))
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



