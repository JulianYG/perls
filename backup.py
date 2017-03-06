
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


# def create_scene(self):
# 	"""
# 	Basic scene needed for running tasks
# 	"""
# 	self.p.resetSimulation()
# 	self.p.setGravity(0, 0, -9.81)
# 	if self.hand:
# 		self.VR_HAND_ID = self.p.loadMJCF("MPL/mpl2.xml")[0]
# 	# self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
# 	self.p.loadURDF("plane.urdf",0,0,0,0,0,0,1)
# 	self.p.loadURDF("table/table.urdf", -1.0,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
# 	self._setup_robot()