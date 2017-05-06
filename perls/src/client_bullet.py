import redis, time
import os, sys, getopt, json
from os.path import join as pjoin
sys.path.append(pjoin(os.getcwd(), 'bullet_'))

from simulation.agents import *
from simulation.interface import *
from simulation.simulator import BulletSimulator
import simulation.utils.helpers as utils
from comm import db

remote_ip = '172.24.68.111'

def run(*args):
	socket = db.RedisComm(remote_ip)

	CONFIG_DIR = pjoin(os.getcwd(), 'configs', args[0] + '.json')
	_CONFIGS = utils.read_config(CONFIG_DIR)

	socket.connect_with_server()
	socket.broadcast_to_server(_CONFIGS)

	interface_type = _CONFIGS['interface']
	if interface_type == 'vr':	# VR interface that takes VR events
		interface = vr_interface.IVR(socket, remote)
	elif interface_type == 'keyboard':	# Keyboard interface that takes keyboard events
		interface = keyboard_interface.IKeyboard(socket, remote)
	elif interface_type == 'cmd':	# Customized interface that takes any sort of command
		interface = cmd_interface.ICmd(socket, remote)
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



