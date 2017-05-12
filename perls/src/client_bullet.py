import redis, time
import os, sys, getopt, json
from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'bullet_'))

from simulation.agents import *
from simulation.interface import *
from simulation.simulator import BulletSimulator
import simulation.utils.helpers as utils
from comm import db

def run(*args):
	
	CONFIG_DIR = pjoin(os.getcwd(), 'bullet_/configs', args[0][0] + '.json')
	_CONFIGS = utils.read_config(CONFIG_DIR)

	socket = db.RedisComm(_CONFIGS['server_ip'])
	socket.connect_with_server()
	socket.broadcast_to_server(_CONFIGS)

	interface_type = _CONFIGS['interface']
	if interface_type == 'vr':	# VR interface that takes VR events
		interface = vr_interface.IVR(socket, True)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = keyboard_interface.IKeyboard(socket, True)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = cmd_interface.ICmd(socket, True)
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



