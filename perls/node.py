import bullet
from bullet.models import *
from bullet.control import *
from bullet.simulator import BulletSimulator
import os, sys, getopt, json
from os.path import join as pjoin
import bullet.util as utils

def build(model, task, socket, filename, record=True):
	"""
	Models still exist in pybullet since they are only used in pybullet
	Example interfacing with ROS:
	in ros_ctrl_IK.py:

	socket = ros_sock(...)
	kuka = kuka.Kuka()
	task = repo['kitchen']
	pybullet_simulator = node.build(kuka, task, socket, 'hi.bin')
	pybullet_simulator.record('path.bin')
	...
	"""
	simulator = BulletSimulator(model, cmd_interface.ICmd(False, socket))
	simulator.setup(task, 0, False)
	if record:
		simulator.run(filename, record=record)
	return simulator

def execute(*args):
	"""
	Default load settings from command line execution. 
	May need a configuration file for this purpose
	"""
	REPO_DIR = pjoin(os.getcwd(), 'data', 'task.json')
	CONFIG_DIR = pjoin(os.getcwd(), 'configs', args[0] + '.json')
	RECORD_LOG_DIR = pjoin(os.getcwd(), 'data', 'record', 'trajectory')

	with open(REPO_DIR, 'r') as f:
		repo = json.loads(f.read())

	_CONFIGS = utils.read_config(CONFIG_DIR)

	interface = _CONFIGS['interface']
	model = _CONFIGS['model']
	job = _CONFIGS['job']
	video = _CONFIGS['video']
	delay = _CONFIGS['delay']
	task = _CONFIGS['task']
	remote = _CONFIGS['remote']
	host = _CONFIGS['host']
	socket = _CONFIGS['socket']
	fixed = _CONFIGS['fixed_gripper_orn']
	force_sensor = _CONFIGS['enable_force_sensor']
	init_pos = _CONFIGS['tool_positions']
	camera_info = _CONFIGS['camera']

	record_file = _CONFIGS['record_file_name']
	replay_file = _CONFIGS['replay_file_name']

	fn = record_file
	if not record_file:
		fn = '_'.join([interface, model, task])

	if model == 'kuka':
		# Change Fixed to True for keyboard
		model = kuka.Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif model == 'sawyer':
		model = sawyer.Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif model == 'pr2':
		model = pr2.PR2(init_pos, enableForceSensor=force_sensor)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')
	
	if interface == 'vr':
		interface = vr_interface.IVR(host, remote)
		vr = True
	elif interface == 'keyboard':
		interface = keyboard_interface.IKeyboard(host, remote)
		vr = False
	elif interface == 'cmd':
		interface = cmd_interface.ICmd(socket, remote=remote)
		vr = False
	else:
		raise NotImplementedError('Non-supported interface.')
	
	simulator = BulletSimulator(model, interface)

	if remote and (job == 'record' or job == 'run'):
		vr = False

	if job == 'record':
		simulator.setup(repo[task], 0, vr)
		simulator.run(fn, True, video)
	elif job == 'replay':
		# Default view point setting
		simulator.set_camera_view(*camera_info)
		if os.path.isfile(pjoin(RECORD_LOG_DIR, replay_file)):
			simulator.setup(repo[task], 1, vr)
			simulator.playback(fn, delay)
		else:
			raise IOError('Record file not found.')
	elif job == 'run':
		simulator.setup(repo[task], 0, vr)
		simulator.run()
	else:
		raise NotImplementedError('Invalid input: Job not recognized.')
	return simulator

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
	execute(config)

if __name__ == '__main__':
	main(sys.argv[1:])



