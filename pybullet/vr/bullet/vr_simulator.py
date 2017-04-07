import time, os
import pybullet as p
from os.path import join as pjoin
import struct

class Simulator(object):

	def __init__(self, model):
		# Default settings for camera
		self.FOCAL_POINT = (0., 0., 0.)
		self.YAW = 35.
		self.PITCH = 50.
		self.ROLL = 0.
		self.FOCAL_LENGTH = 5.
		self.UP_AX_IDX = 2
		self.viewMatrix = None
		self.projectionMatrix = None

		self._model = model
		self.VIDEO_DIR = pjoin(os.getcwd(), 'data', 'video')
		self.RECORD_LOG_DIR = pjoin(os.getcwd(), 'data', 'record')

	def load_task(self, task, flag):

		if not self._model.reset(flag):
			raise Exception('Cannot detect running VR application. Please try again.')
		self._model.load_task(task)
		self._control_map = self._model.create_control_mappings()

	def record(self, file, video=False):
		try:
			logIds = []
			if video:
				# Does logging only need to be called once with SharedMemory? 
				logIds.append(p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 
					pjoin(self.VIDEO_DIR, file + '.mp4')))
			else:
				# Record everything
				logIds.append(p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
					pjoin(self.RECORD_LOG_DIR, 'generic.' + file)))
				# ctrlLog = p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS, 
				# 	file + '_ctrl')
			while True:
				events = p.getVREvents()
				skip_flag = self._model.is_unicontrol()
				for e in (events):
					if skip_flag:
						if e[0] == self._model.controllers[1]:
							break
						self._model.control(e, self._control_map)
					else:
						self._model.control(e, self._control_map)

		except KeyboardInterrupt:
			self.quit(logIds)

	def replay(self, file, delay=0.0001):
		p.resetDebugVisualizerCamera(cameraDistance=self.FOCAL_LENGTH, 
			cameraYaw=self.YAW, cameraPitch=self.PITCH, 
			cameraTargetPosition=self.FOCAL_POINT)
		log = self.parse_log(pjoin(self.RECORD_LOG_DIR, 'generic.' + file), verbose=True)
		self.replay_log(log, delay=delay)
		self.quit([])

	def set_camera_view(self, targetPosX, targetPosY, targetPosZ, roll, pitch, yaw, dist):
		"""
		Set the view of camera; typically egocentric, or oblique
		"""
		self.FOCAL_POINT = (targetPosX, targetPosY,  targetPosZ)
		self.PITCH = pitch
		self.ROLL = roll
		self.YAW = yaw
		self.FOCAL_LENGTH = dist
		self.viewMatrix = p.computeViewMatrixFromYawPitchRoll((targetPosX, targetPosY, targetPosZ), 
			dist, yaw, pitch, roll, self.UP_AX_IDX)
		self.projectionMatrix = p.computeProjectionMatrixFOV(60, 600 / 540., .01, 1000.)

	def video_capture(self):
		"""
		This is very, very slow at the moment
		"""
		img_arr = p.getCameraImage(600, 540, self.viewMatrix, self.projectionMatrix)
		np_img = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4)) / 255.

		plt.imshow(np_img)
		plt.pause(0.001)

	def parse_log(self, filename, verbose=True):

	  	f = open(filename, 'rb')
	  	print('Opened'),
	  	print(filename)

	  	keys = f.readline().decode('utf8').rstrip('\n').split(',')
	  	fmt = f.readline().decode('utf8').rstrip('\n')

	  	# The byte number of one record
	  	sz = struct.calcsize(fmt)
	  	# The type number of one record
	  	ncols = len(fmt)

	  	if verbose:
	  		print('Keys:'), 
	  		print(keys)
	  		print('Format:'),
	  		print(fmt)
	  		print('Size:'),
	  		print(sz)
	  		print('Columns:'),
	  		print(ncols)

	  	# Read data
	  	wholeFile = f.read()
	  	# split by alignment word
	  	chunks = wholeFile.split(b'\xaa\xbb')
	  	log = list()
	  	for chunk in chunks:
		    if len(chunk) == sz:
		      	values = struct.unpack(fmt, chunk)
		      	record = list()
		      	for i in range(ncols):
		        	record.append(values[i])
		      	log.append(record)

	  	return log

	def replay_log(self, log, delay=0.0005):

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

	def quit(self, fp): # logId
		if isinstance(fp, list):
			for Id in fp:
				p.stopStateLogging(Id)
		else:
			fp.close()

		p.resetSimulation()
		p.disconnect()


