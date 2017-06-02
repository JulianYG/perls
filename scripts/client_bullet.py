import redis, time
import os, sys, getopt, json
from os.path import join as pjoin

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/bullet_')))

from simulation.interface import IVR, IKeyboard, ICmd
from simulation.simulator import BulletSimulator

import simulation.utils.helpers as utils
from comm import db

def run(*args):
	
	CONFIG_DIR = pjoin(os.getcwd(), '../src/bullet_/configs', args[0][0] + '.json')
	_CONFIGS = utils.read_config(CONFIG_DIR)

	socket = db.RedisComm(_CONFIGS['server_ip'])
	socket.connect_with_server()
	socket.broadcast_to_server(_CONFIGS)

	interface_type = _CONFIGS['interface']
	if interface_type == 'vr':	# VR interface that takes VR events
		interface = IVR(socket, True)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = IKeyboard(socket, True)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = ICmd(socket, True)
	else:
		raise NotImplementedError('Non-supported interface.')

	# Singleton simulator
	simulator = BulletSimulator(None, interface, None, None)
	try:
		simulator.run_as_client()
	except KeyboardInterrupt:
		simulator.quit()

if __name__ == '__main__':
	run(sys.argv[1:])



