import os, struct, time
from datetime import datetime
from os.path import join as pjoin
from robot import Robot

class RobotController(object):

	def __init__(self, agent, interface, task, scene, gui=True, vr=False, log_dir=''):
		# Default settings for camera

		log_dir = log_dir or pjoin(os.getcwd(), 'log')
		self.VIDEO_DIR = pjoin(log_dir, 'video')
		self.TRAJECTORY_LOG_DIR = pjoin(log_dir, 'trajectory')
		self.CONTROL_LOG_DIR = pjoin(log_dir, 'control')
		self.CONTACT_LOG_DIR = pjoin(log_dir, 'contact')

	def set_time_step(self, time_step):
		pass

	def step_control(self):
		pass

	def run_as_server(self, file='', record=False, video=False):
		pass

	def playback(self, file, delay=0.0001):
		pass

	def set_camera_view(self, targetPosX, targetPosY, targetPosZ, roll, 
		pitch, yaw, dist, width=600, height=540):
		"""
		Set the view of camera; typically egocentric, or oblique
		"""
		pass

	def snapshot(self, show=False):
		pass

	def quit(self):
		"""
		Perform clean up before exit
		"""
		pass

	def _record(self, filename, video=False):
		pass

	def _replay(self, log, delay=0.0005):

		pass

	def _setup(self, flag):
		pass