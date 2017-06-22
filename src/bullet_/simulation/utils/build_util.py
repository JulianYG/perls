import pybullet as p
import sys, os
import json
from os.path import join as pjoin
sys.path.append(os.path.abspath(pjoin(os.path.dirname(__file__), '..')))

from comm import db

__package__ = 'bullet_.simulation'

from .tool import PR2
from .arm import Kuka, Sawyer
from .interface import IVR, IKeyboard, ICmd
from .simulator import BulletSimulator
from .utils import io
from .utils.misc import Constant

_config_keys = [
	'interface',
	'agent',
	'video',
	'delay',
	'task',
	'server_ip',
	'fixed_gripper_orn',
	'enable_force_sensor',
	'tool_positions',
	'camera',
	'gui',
	'scene',
	'record_file_name',
	'replay_file_name',
]

def build_by_config(configs, work_dir, remote=False):
	
	task_dir = pjoin(work_dir, 'configs', 'task.json')
	scene_dir = pjoin(work_dir, 'configs', 'scene.json')
	record_dir = pjoin(work_dir, 'log', 'record', 'trajectory')

	with open(task_dir, 'r') as f:
		task_repo = json.loads(f.read())
	with open(scene_dir, 'r') as f:
		scene_repo = json.loads(f.read())

	interface_type = configs.get('interface', '')
	agent = configs.get('agent', '')
	video = configs.get('video', '')
	task = configs.get('task', '')
	ip = configs.get('server_ip', '')
	fixed = configs.get('fixed_gripper_orn', False)
	force_sensor = configs.get('enable_force_sensor', False)
	init_pos = configs.get('tool_positions', [0.,])
	camera_info = configs.get('camera', None)
	gui = configs.get('gui', True)
	scene = configs.get('scene', '')

	if agent == 'kuka':
		# Change Fixed to True for keyboard
		agent = Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'sawyer':
		agent = Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif agent == 'pr2':
		agent = PR2(init_pos, enableForceSensor=force_sensor)
	else:
		print('Agent registered as Nonetype.')
		agent = None

	socket = db.RedisComm(ip) if remote else None
	
	if interface_type == 'vr':	# VR interface that takes VR events
		interface = IVR(socket, remote, task)
		vr = True
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = IKeyboard(socket, remote, task)
		vr = False
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = ICmd(socket, remote, task)
		vr = False
	else:
		print('Interface registered as Nonetype.')
		interface = None
		vr = False

	simulator = BulletSimulator(agent, interface, 
								task_repo[task], scene_repo[scene],
								gui=gui, vr=vr, log_dir=record_dir)
	simulator.set_camera_view(*camera_info)
	return simulator


	