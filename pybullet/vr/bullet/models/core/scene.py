import pybullet as p

class Scene(object):
	"""
	The basic scene setup in VR 
	"""
	def __init__(self, enableForceSensor, pos):
		"""
		Other subclasses may re-implement the constructor
		"""		
		self.BUTTONS = 6
		self.ORIENTATION = 2
		self.controllers = None
		self.side_obj_cnt = 0

		self.hand = False
		self.grippers = []
		self.constraints = []
		self.pos = pos
		self.arms = []
		self.env_obj = []
		self.VR_HAND_ID = None

		self.solo = None
		self.has_force_sensor = enableForceSensor
		self.MAX_FORCE = 500

	def reset(self, replay, vr):
		"""
		Load task for both recording and replay
		"""
		try:
			# Use GUI for replay or non-vr interface
			if replay or not vr:
				p.connect(p.GUI)
			else:
				p.connect(p.SHARED_MEMORY)

			# In order to avoid real time simulation in replay
			# for deterministic paths
			if replay:
				p.setRealTimeSimulation(0)
			else:
				p.setRealTimeSimulation(1)
			
			p.setInternalSimFlags(0)
			# convenient for video recording
		except p.error:
			return 0
			
		self.controllers = [e[0] for e in p.getVREvents()]
		self.setup_tools()
		self.solo = len(self.arms) == 1 or len(self.grippers) == 1
		return 1

	def setup_tools(self):
		p.resetSimulation()
		self._load_tools(self.pos)
		
	def setup_scene(self, task):
		raise NotImplementedError("Each VR model must re-implement this method.")

	def get_env(self):
		return self.env_obj

	def set_time_step(self, time_step):
		p.setRealTimeSimulation(0)
		p.setTimeStep(time_step)

	def step_simulation(self, time_step):
		p.stepSimulation()

	def load_min_env(self):
		p.loadURDF("plane.urdf",0,0,0,0,0,0,1)

	def load_basic_env(self):

		p.loadURDF("plane.urdf",0,0,0,0,0,0,1)
		p.loadURDF("table/table.urdf", 1.1, -0.2, 0., 0., 0., 0.707107, 0.707107)

	def load_default_env(self):

		p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
		p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)
		p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
		p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)
		p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)
		p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)
		p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)
		p.loadURDF("teddy_vhacd.urdf", -0.100000,-2.200000,0.850000,0.000000,0.000000,0.000000,1.000000)
		p.loadURDF("sphere_small.urdf", -0.100000,-2.255006,1.169706,0.633232,-0.000000,-0.000000,0.773962)
		p.loadURDF("cube_small.urdf", 0.300000,-2.100000,0.850000,0.000000,0.000000,0.000000,1.000000)
		p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)
		shelf = p.loadSDF("kiva_shelf/model.sdf")[0]
		p.resetBasePositionAndOrientation(shelf, [-0.700000,-2.200000,1.204500],[0.000000,0.000000,0.000000,1.000000])

	def _load_task(self, task):
		for obj_pose in task:
			if len(obj_pose) == 1:
				p.loadSDF(obj_pose[0])
			else:
				p.loadURDF(*obj_pose)

	def _load_tools(self, pos):
		raise NotImplementedError("Each VR model must re-implement this method.")



