import pybullet as p
import numpy as np
import os, struct, time
from datetime import datetime
from time import strftime
from matplotlib import pyplot as plt
from os.path import join as pjoin
from bullet.utils import helpers as utils

class BulletSimulator(object):

	def __init__(self, agent, interface, task, scene, gui=True, vr=False, log_dir=''):
		# Default settings for camera
		self.FOCAL_POINT = (0., 0., 0.)
		self.YAW = 35.
		self.PITCH = 50.
		self.ROLL = 0.
		self.FOCAL_LENGTH = 5.
		self.UP_AX_IDX = 2
		self.viewMatrix = None
		self.projectionMatrix = None
		self.vr = vr
		self.gui = gui
		self.logIds = []
		self.task = task
		self.scene = scene
		self._interface = interface
		self.agent = agent

		log_dir = log_dir or pjoin(os.getcwd(), 'log')
		self.VIDEO_DIR = pjoin(log_dir, 'video')
		self.TRAJECTORY_LOG_DIR = pjoin(log_dir, 'trajectory')
		self.CONTROL_LOG_DIR = pjoin(log_dir, 'control')
		self.CONTACT_LOG_DIR = pjoin(log_dir, 'contact')

	def set_time_step(self, time_step):
		p.setRealTimeSimulation(0)
		p.setTimeStep(time_step)

	def step_simulation(self):
		p.stepSimulation()

	def run(self, file='', record=False, video=False, remote_render=False):
		self._setup(0)
		try:
			if record:
				file += '_' + datetime.now().strftime('%m-%d-%H-%M-%S')
				self._record(file, video)
			# if remote_render:
			# 	self._interface.client_communicate(self.agent)
			self._interface.communicate(self.agent, self.scene, self.task, self.gui)
		except (KeyboardInterrupt, SystemExit) as e:
			self.quit()		

	def playback(self, file, delay=0.0001):
		self._setup(1)
		p.resetDebugVisualizerCamera(cameraDistance=self.FOCAL_LENGTH, 
			cameraYaw=self.YAW, cameraPitch=self.PITCH, 
			cameraTargetPosition=self.FOCAL_POINT)
		log = utils.parse_log(pjoin(self.TRAJECTORY_LOG_DIR, 'traj.' + file), verbose=True)
		self._replay(log, delay=delay)
		self.quit()

	def set_camera_view(self, targetPosX, targetPosY, targetPosZ, roll, 
		pitch, yaw, dist, width=600, height=540):
		"""
		Set the view of camera; typically egocentric, or oblique
		"""
		self.FOCAL_POINT = (targetPosX, targetPosY,  targetPosZ)
		self.PITCH = pitch
		self.ROLL = roll
		self.YAW = yaw
		self.FOCAL_LENGTH = dist
		self.image_width = width
		self.image_height = height
		self.viewMatrix = p.computeViewMatrixFromYawPitchRoll((targetPosX, targetPosY, targetPosZ), 
			dist, yaw, pitch, roll, self.UP_AX_IDX)
		self.projectionMatrix = p.computeProjectionMatrixFOV(60, float(width) / height, .01, 1000.)

	def snapshot(self, show=False):
		img_arr = p.getCameraImage(self.image_width, self.image_height, 
			self.viewMatrix, self.projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.
		if show:
			plt.imshow(np_img)
		return np_img

	def quit(self):
		"""
		Perform clean up before exit
		"""
		if self.logIds:
			for Id in self.logIds:
				p.stopStateLogging(Id)
		if self._interface:
			self._interface.close()
		p.resetSimulation()
		p.disconnect()

	def _record(self, filename, video=False):
		if video:
			if self.gui:
				# Does logging only need to be called once with SharedMemory? 
				self.logIds.append(p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 
					pjoin(self.VIDEO_DIR, filename + '.mp4')))
			else:
				print('Video cannot be saved without simulation in GUI. Please check your config')
		else:
			# Record everything
			self.logIds.append(p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
				pjoin(self.TRAJECTORY_LOG_DIR, 'traj.' + filename)))
			self.logIds.append(p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS, 
				pjoin(self.CONTROL_LOG_DIR, 'ctrl.' + filename)))
			self.logIds.append(p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS,
				pjoin(self.CONTACT_LOG_DIR, 'cont.' + filename)))

	def _replay(self, log, delay=0.0005):

		for record in log:
			time_stamp = float(record[1])
			obj = record[2]
			pos = record[3: 6]
			orn = record[6: 10]
			p.resetBasePositionAndOrientation(obj, pos, orn)
			numJoints = p.getNumJoints(obj)
			for i in range(numJoints):
				jointInfo = p.getJointInfo(obj, i)
				qIndex = jointInfo[3]
				if qIndex > -1:
					p.resetJointState(obj, i, record[qIndex - 7 + 17])
			time.sleep(delay)

	def _setup(self, flag):
		if not os.path.exists(self.VIDEO_DIR):
			os.makedirs(self.VIDEO_DIR)
		if not os.path.exists(self.TRAJECTORY_LOG_DIR):
			os.makedirs(self.TRAJECTORY_LOG_DIR)
		if not os.path.exists(self.CONTROL_LOG_DIR):
			os.makedirs(self.CONTROL_LOG_DIR)	
		if not os.path.exists(self.CONTACT_LOG_DIR):
			os.makedirs(self.CONTACT_LOG_DIR)
		try:
			# Use GUI for replay or non-vr interface
			if flag or not self.vr:
				if self.gui:
					p.connect(p.GUI)
					# Disable sidebar and shadows
					p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
					p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
				else:
					p.connect(p.DIRECT)
			else:
				p.connect(p.SHARED_MEMORY)

			# In order to avoid real time simulation in replay
			# for deterministic paths
			if flag:
				p.setRealTimeSimulation(0)
			else:
				p.setRealTimeSimulation(1)
			
			p.setInternalSimFlags(0)
			# convenient for video recording
		except p.error:
			if self.vr:
				raise Exception('Cannot detect running VR application. Please try again.')
			else:
				raise Exception('Cannot create pybullet GUI instance. Please try again.')
		p.resetSimulation()
		self.agent.setup_scene(self.scene, self.task, self.gui)


