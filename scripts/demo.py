#!/usr/bin/env python

import os, sys, getopt, json
from os.path import join as pjoin

path = os.path.dirname(os.path.abspath(__file__))
rpath = os.path.normpath(pjoin(path, '..'))
sys.path.append(pjoin(path, '../src'))

from bullet_ import simulation

__package__ = 'sim_.simulation'

from .utils import io, build_util

def execute(*args):
	"""
	Default load settings from command line execution. 
	May need a configuration file for this purpose
	"""
	WORK_DIR = args[1]
	CONFIG_DIR = pjoin(WORK_DIR, 'configs', args[0] + '.json')
	_CONFIGS = io.read_config(CONFIG_DIR)
	
	job = _CONFIGS['job']
	record_file = _CONFIGS['record_file_name']
	replay_file = _CONFIGS['replay_file_name']
	video = _CONFIGS['video']

	fn = record_file or '_'.join([interface_type, agent, task])

	simulator = build_util.build_by_config(_CONFIGS, WORK_DIR)

	if job == 'record':
		simulator.run_as_server(fn, True, video)
	elif job == 'replay':
		# Default view point setting
		delay = _CONFIGS['delay']
		simulator.playback(replay_file, delay)
	elif job == 'run':
		simulator.run_as_server()
	else:
		raise NotImplementedError('Invalid input: Job not recognized.')

def usage():
	print('Please specify the configuration file under ./configs. Default config: 1')
	print('Usage: python node.py -c [config]')

def main(argv):

	config = '1'	# default config
	try:
		opts, args = getopt.getopt(argv, 'hc:', ['help', 'config='])
	except getopt.GetoptError:
		usage()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			usage()
			sys.exit(0)
		elif opt in ('-c', '--config'):
			config = arg

	wd = pjoin(rpath, 'src/sim_')

	execute(config, wd)

if __name__ == '__main__':
	main(sys.argv[1:])



