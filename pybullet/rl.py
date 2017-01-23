# import pybullet as p
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
import numpy as np

class BulletPhysicsVR(object):

	def __init__(self, pybullet, kuka_id, task=0):
		self.tasks = self._init_task()
		self.task = task
		self.p = pybullet

		self.CONTROLLER_ID = 0
		self.KUKA_GRIPPER_ID = kuka_id
		self.POSITION = 1
		self.BUTTONS = 6

		self.FOCAL_POINT = (0., 0., 0.)
		self.YAW = 35.
		self.PITCH = 50.
		self.ROLL = 0.
		self.FOCAL_LENGTH = 5.
		self.UP_AX_IDX = 2

		self.OBJ_CNT = self.KUKA_GRIPPER_ID + 1
		self.tracking_obj = None
		self.viewMatrix = None
		self.projectionMatrix = None

	def _load_task(self, flag):
		try:
			if flag:
				self.p.connect(self.p.GUI)
				self.p.setInternalSimFlags(0)
				self.p.setRealTimeSimulation(0)
			else:
				self.p.connect(self.p.SHARED_MEMORY)
				self.p.setInternalSimFlags(1)
				self.p.setRealTimeSimulation(1)
		except self.p.error:
			return 0
		self._init_scene()
		seq = self.tasks[self.task]
		curr_idx = self.p.getNumBodies() + self.OBJ_CNT
		ob_s, ob_e = seq[0]
		self.tracking_obj = range(curr_idx + ob_s, curr_idx + ob_e + 1)
		for t in seq[1:]:
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

			while True:
				events = self.p.getVREvents()

				for e in (events):
					# If the user think one task is completed
					if (e[self.BUTTONS][1] & self.p.VR_BUTTON_WAS_TRIGGERED):
							# self.p.resetSimulation()
							# self.p.removeAllUserDebugItems()
						self.p.addUserDebugText('good job!', (1.7, 0, 1), (255, 0, 0), 12, 10)
						# Can add line for mark here		

				# Saving events routine
				if events and events[0][5] > 0:	# Only take record when moving events happen
				# Removing this line for video generation may sync the process?
					curr_time = time.time()
					time_elapse = curr_time - prev_time
					prev_time = curr_time
					for o_id in self.tracking_obj:
						row = [(o_id)] + list(self.p.getBasePositionAndOrientation(o_id)[0]) + list(self.p.getBasePositionAndOrientation(o_id)[1])
						writer.writerow(row)
					gripper_info = [self.KUKA_GRIPPER_ID] + list(self.p.getBasePositionAndOrientation(self.KUKA_GRIPPER_ID)[0]) + \
						list(self.p.getBasePositionAndOrientation(self.KUKA_GRIPPER_ID)[1])
					delay = [-1, time_elapse]
					writer.writerow(gripper_info)
					writer.writerow(delay)

		except KeyboardInterrupt:
			self._exit_routine(f)

	def replay(self, file, saveVideo=0):
		load_status = 0
		while load_status == 0:
			load_status = self._load_task(1)

		self.p.setCameraViewPoint(self.FOCAL_POINT[0], self.FOCAL_POINT[1], self.FOCAL_POINT[2], 
			self.PITCH, self.YAW, self.FOCAL_LENGTH)

		r, g = self._setup_robot()
		f = open(file, 'r')
		reader = csv.reader(f)
		delay = 0
		for row in reader:
			if int(row[0]) == -1:
				delay = float(row[1]) / 12
			else:
				time.sleep(delay)
				if int(row[0]) != self.KUKA_GRIPPER_ID:
					self.p.resetBasePositionAndOrientation(int(row[0]) - self.KUKA_GRIPPER_ID - 1, (float(row[1]), float(row[2]), 
						float(row[3])), (float(row[4]), float(row[5]), float(row[6]), float(row[7])))
				else:
					eef_pos = (float(row[1]), float(row[2]), float(row[3]))
					eef_orien = (float(row[4]), float(row[5]), float(row[6]), float(row[7]))
					joint_pos = self.p.calculateInverseKinematics(r, 6, eef_pos, eef_orien)
					self.p.resetBasePositionAndOrientation(g[0], eef_pos, eef_orien)
					for i in range(len(joint_pos)):
						self.p.resetJointState(r, i, joint_pos[i])
				if saveVideo:
					self.video_capture()
		self._exit_routine(f)


	def video_capture(self):

		# viewMatrix = self.p.computeViewMatrixFromYawPitchRoll((targetPosX, targetPosY, targetPosZ), dist, yaw, pitch, roll, 2)

		# projectionMatrix = self.p.computeProjectionMatrixFOV(60, 600 / 540., .01, 1000.)
		img_arr = self.p.getCameraImage(600, 540, self.viewMatrix, self.projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.

		plt.imshow(np_img)
		plt.pause(0.001)

	def set_camera_view(self, targetPosX, targetPosY, targetPosZ, roll, pitch, yaw, dist):
		
		self.FOCAL_POINT = (targetPosX, targetPosY,  targetPosZ)
		self.PITCH = pitch
		self.ROLL = roll
		self.YAW = yaw
		self.FOCAL_LENGTH = dist
		self.viewMatrix = self.p.computeViewMatrixFromYawPitchRoll((targetPosX, targetPosY, targetPosZ), dist, yaw, pitch, roll, 2)
		self.projectionMatrix = self.p.computeProjectionMatrixFOV(60, 600 / 540., .01, 1000.)


	def _init_scene(self):

		self.p.resetSimulation()
		self.p.setGravity(0,0,-9.81)
		self.p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		# self.p.loadURDF("jenga/jenga.urdf", 1.300000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
		# self.p.loadURDF("jenga/jenga.urdf", 1.200000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
		# self.p.loadURDF("jenga/jenga.urdf", 1.100000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)

	def _setup_robot(self):
		self.p.loadURDF("plane.urdf",0,0,0,0,0,0,1)
		robot = self.p.loadURDF('kuka_iiwa/model_vr_limits.urdf', 1.4,-0.2,0.6,0,0,0,1)	
		gripper = self.p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')
		return robot, gripper

	def _init_task(self):
		repo = {}
		repo[0] = [(3,7), ("cvgl/pole.urdf",0.80000,0.100000,0.699990,0.00000,0.0,0.00000,1), 
			("cvgl/pole.urdf",0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1),
			("cvgl/pole.urdf",0.80000,-0.500000,0.699990,0.00000,0.0,0.00000,1),
			("cvgl/torus_0.urdf",0.8,0.1,0.69999,1,0,0,1),
			("cvgl/torus_1.urdf",0.8,0.1,0.74999,1,0,0,1),
			("cvgl/torus_2.urdf",0.8,0.1,0.79999,1,0,0,1),
			("cvgl/torus_3.urdf",0.8,0.1,0.84999,1,0,0,1),
			("cvgl/torus_4.urdf",0.8,0.1,0.89999,0,0,0,1)]
		return repo

	def _exit_routine(self, fp):
		fp.close()
		self.p.resetSimulation()
		self.p.disconnect()

# b = BulletPhysicsVR(p, 2)
# b.record('see.csv')
# b.replay('see.csv')


# def set_camera_position():
# 	# upAxisIndex: 1 for Y, 2 for Z
# 	pass

# def render_scene_full(intvl=10):
# 	for pitch in range(0, 360, intvl):
# 		viewMatrix = p.computeViewMatrixFromYawPitchRoll(FOCAL_POINT, FOCAL_LENGTH, YAW, pitch, ROLL, UP_AX_IDX)
# 		projectionMatrix = p.computeProjectionMatrixFOV(60, 320 / 240., 0.01, 1000.)
# 		img_arr = p.getCameraImage(320, 240, viewMatrix, projectionMatrix)
# 		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.
# 		plt.imshow(np_img)
# 		plt.pause(0.01)

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
