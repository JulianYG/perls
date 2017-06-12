
from collections import defaultdict
import os, sys, getopt, json
from os.path import join as pjoin

path = os.path.dirname(os.path.abspath(__file__))
rpath = os.path.normpath(pjoin(path, '..'))
sys.path.append(pjoin(path, '../src'))

from bullet_ import simulation
from comm import db

__package__ = 'bullet_.simulation'

from .tool import PR2
from .arm import Sawyer, Kuka
from .interface import IVR, IKeyboard, ICmd
from .simulator import BulletSimulator
from .utils import build_util

bullet_path = pjoin(rpath, 'src/bullet_')
# sys.path.append(bullet_path)

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

	job = config['job']

	simulator = build_util.build_by_config(config, bullet_path)

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



