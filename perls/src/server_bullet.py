import socket
import fcntl
import struct
from collections import defaultdict
import os, sys, getopt, json
from os.path import join as pjoin

bullet_path = pjoin(os.getcwd(), 'bullet_')
sys.path.append(bullet_path)

from simulation.agents import *
from simulation.interface import *
from simulation.simulator import BulletSimulator

from simulation.utils import helpers as utils
from comm import db

TASK_DIR = pjoin(bullet_path, 'configs', 'task.json')
SCENE_DIR = pjoin(bullet_path, 'configs', 'scene.json')
RECORD_LOG_DIR = pjoin(bullet_path, 'log', 'record', 'trajectory')

# def get_ip_address(ifname):
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     return socket.inet_ntoa(fcntl.ioctl(
#         s.fileno(),
#         0x8915,  # SIOCGIFADDR
#         struct.pack('256s', ifname[:15])
#     )[20:24])

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
		agent = kuka.Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'sawyer':
		agent = sawyer.Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'pr2':
		agent = pr2.PR2(init_pos, enableForceSensor=force_sensor)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')

	host = db.RedisComm('localhost', port=6379)  # ip

	if interface_type == 'vr':	# VR interface that takes VR events
		interface = vr_interface.IVR(host, True)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = keyboard_interface.IKeyboard(host, True)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = cmd_interface.ICmd(host, True)
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



