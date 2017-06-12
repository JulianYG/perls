import redis, time
import os, sys, getopt, json
from os.path import join as pjoin

from demo import execute

path = os.path.dirname(os.path.abspath(__file__))
rpath = os.path.normpath(pjoin(path, '..'))
sys.path.append(pjoin(path, '../src'))

from comm import db
from bullet_ import simulation

__package__ = 'bullet_.simulation'

from .interface import IVR, IKeyboard, ICmd
from .simulator import BulletSimulator
from .utils import io, build_util

def run(*args):

	CONFIG_DIR = pjoin(rpath, 'src/bullet_/configs', args[0][0] + '.json')
	_CONFIGS = io.read_config(CONFIG_DIR)

	simulator = build_util.build_by_config(_CONFIGS, 
		pjoin(rpath, 'src/bullet_'), remote=True)

	try:
		simulator.run_as_client(_CONFIGS)
	except KeyboardInterrupt:
		simulator.quit()

if __name__ == '__main__':
	run(sys.argv[1:])



