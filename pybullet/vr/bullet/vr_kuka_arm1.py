from os.path import join as pjoin
from core.vr_kuka import KukaVR

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
					pjoin(self.VIDEO_DIR, file + '.mp4'))
			else:
				# Record everything
				bodyLog = self.p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file))
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


