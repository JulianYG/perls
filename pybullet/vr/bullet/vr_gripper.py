from os.path import join as pjoin
from bullet.core.vr_physics import BulletVR

class PR2GripperVR(BulletVR):

	#TODO: separate loading grasp task with kuka arm loading

	
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
		logIds = []
		try:
			if video:
				# Does logging only need to be called once with SharedMemory? 
				logIds.append(self.p.startStateLogging(self.p.STATE_LOGGING_VIDEO_MP4, 
					pjoin(self.VIDEO_DIR, file + '.mp4')))

			else:
				# Record everything
				logIds.append(self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file)))
				# ctrlLog = self.p.startStateLogging(self.p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')
			
			while True:

				events = self.p.getVREvents()
				for e in (events):

					# Only use one controller
					if e[0] == self.controllers[1]:
						break

					# PR2 gripper follows VR controller				
					self.p.changeConstraint(self.pr2_cid, e[1], e[self.ORIENTATION], maxForce=500)	

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



