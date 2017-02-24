
class BulletPhysicsVR(object):

	def __init__(self, pybullet, task):
		"""
		Other subclasses may re-implement the constructor
		"""
		self.task = task
		self.p = pybullet
		
		self.BUTTONS = 6
		self.ORIENTATION = 2
		self.controllers = [3, 4]

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

	def quit(self, fp):
		fp.close()
		self.p.resetSimulation()
		self.p.disconnect()

