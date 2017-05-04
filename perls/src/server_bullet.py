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
from simulation.comm import *

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
	TASK_DIR = pjoin(bullet_path, 'configs', 'task.json')
	SCENE_DIR = pjoin(bullet_path, 'configs', 'scene.json')
	RECORD_LOG_DIR = pjoin(bullet_path, 'log', 'record', 'trajectory')

	socket = db.RedisComm('localhost', port=6379)  # ip

	socket.connect_with_client()

	_CONFIGS = {}

	while not _CONFIGS:
		for event in socket.listen_to_client():
			e = eval(event)
			# Make sure it's config
			if isinstance(e, dict) and 'task' in e:
				_CONFIGS = e

	with open(TASK_DIR, 'r') as f:
		task_repo = json.loads(f.read())
	with open(SCENE_DIR, 'r') as f:
		scene_repo = json.loads(f.read())

	interface_type = _CONFIGS['interface']
	agent = _CONFIGS['agent']
	job = _CONFIGS['job']
	video = _CONFIGS['video']
	delay = _CONFIGS['delay']
	task = _CONFIGS['task']
	remote = _CONFIGS['remote']
	ip = _CONFIGS['server_ip']
	fixed = _CONFIGS['fixed_gripper_orn']
	force_sensor = _CONFIGS['enable_force_sensor']
	init_pos = _CONFIGS['tool_positions']
	camera_info = _CONFIGS['camera']
	server = _CONFIGS['server']
	gui = _CONFIGS['gui']
	scene = _CONFIGS['scene']

	record_file = _CONFIGS['record_file_name']
	replay_file = _CONFIGS['replay_file_name']

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

	if interface_type == 'vr':	# VR interface that takes VR events
		interface = vr_interface.IVR(socket, remote)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = keyboard_interface.IKeyboard(socket, remote)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = cmd_interface.ICmd(socket, remote)
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
	return simulator

def usage():
	print('Please specify the configuration file under ./configs. Default config: 1')
	print('Usage: python node.py -c [config]')

if __name__ == '__main__':
	execute()



