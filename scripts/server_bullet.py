
from collections import defaultdict
import os, sys, getopt, json
from os.path import join as pjoin

path = os.path.abspath(os.getcwd()).rsplit('/')
rpath = '/'.join(path[: path.index('perls') + 1])
sys.path.append(pjoin(rpath, 'src'))

from bullet_ import simulation
from comm import db

__package__ = 'bullet_.simulation'

from .tool import PR2
from .arm import Sawyer, Kuka
from .interface import IVR, IKeyboard, ICmd
from .simulator import BulletSimulator

bullet_path = pjoin(rpath, 'src/bullet_')
sys.path.append(bullet_path)
TASK_DIR = pjoin(bullet_path, 'configs', 'task.json')
SCENE_DIR = pjoin(bullet_path, 'configs', 'scene.json')
RECORD_LOG_DIR = pjoin(bullet_path, 'log', 'record', 'trajectory')

def execute():
	"""
	Default load settings from command line execution. 
	May need a configuration file for this purpose
	"""
	
	socket = db.RedisComm('localhost', port=6379)  # ip
	socket.connect_with_client()

	try:
		while 1:
			_CONFIGS = {}
			while not _CONFIGS:
				for event in socket.listen_to_client():
					e = eval(event)
					# Make sure it's config
					if isinstance(e, dict) and 'task' in e:
						_CONFIGS = e
			run_server(_CONFIGS)
	except KeyboardInterrupt:
		socket.disconnect()
		sys.exit(0)

def run_server(config):

	with open(TASK_DIR, 'r') as f:
		task_repo = json.loads(f.read())
	with open(SCENE_DIR, 'r') as f:
		scene_repo = json.loads(f.read())

	interface_type = config['interface']
	agent = config['agent']
	job = config['job']
	video = config['video']
	delay = config['delay']
	task = config['task']
	fixed = config['fixed_gripper_orn']
	force_sensor = config['enable_force_sensor']
	init_pos = config['tool_positions']
	camera_info = config['camera']
	gui = config['gui']
	scene = config['scene']

	record_file = config['record_file_name']
	replay_file = config['replay_file_name']

	fn = record_file
	if not record_file:
		fn = '_'.join([interface_type, agent, task])

	if agent == 'kuka':
		# Change Fixed to True for keyboard
		agent = Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'sawyer':
		agent = Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'pr2':
		agent = PR2(init_pos, enableForceSensor=force_sensor)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')

	host = db.RedisComm('localhost', port=6379)  # ip

	if interface_type == 'vr':	# VR interface that takes VR events
		interface = IVR(host, True)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = IKeyboard(host, True)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = ICmd(host, True)
	else:
		raise NotImplementedError('Non-supported interface.')

	simulator = BulletSimulator(agent, interface, 
								task_repo[task], scene_repo[scene],
								gui=gui)
	if job == 'record':
		simulator.run_as_server(fn, True, video)
	elif job == 'replay':
		# Default view point setting
		simulator.set_camera_view(*camera_info)
		if os.path.isfile(pjoin(RECORD_LOG_DIR, replay_file)):
			simulator.playback(fn, delay)
		else:
			raise IOError('Record file not found.')
	elif job == 'run':
		simulator.run_as_server()
	else:
		raise NotImplementedError('Invalid input: Job not recognized.')
	del simulator
	return 0

def usage():
	print('Please specify the configuration file under ./configs. Default config: 1')
	print('Usage: python node.py -c [config]')

if __name__ == '__main__':
	execute()



