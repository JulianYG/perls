from os.path import join as pjoin
from core.vr_kuka import KukaVR

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
					pjoin(self.VIDEO_DIR, file + '.mp4'))
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file))
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

