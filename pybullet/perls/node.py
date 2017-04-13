import bullet
from bullet.models import *
from bullet.control import *
from bullet.simulator import BulletSimulator
import os, sys, getopt, json
from os.path import join as pjoin

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
	REPO_DIR = './data/task.json'
	RECORD_LOG_DIR = './data/record/trajectory'

	with open(REPO_DIR, 'r') as f:
		repo = json.loads(f.read())
	i, m, j, v, d, t, r, s = args
	fn = '_'.join([i, m, t])
	if m == 'kuka':
		# Change Fixed to True for keyboard
		model = kuka.Kuka([0.3, -0.5], fixed=True, enableForceSensor=False)
	elif m == 'sawyer':
		model = sawyer.Sawyer([0.0], fixed=True, enableForceSensor=False)
	elif m == 'pr2':
		model = pr2.PR2([0.3, -0.5], enableForceSensor=False)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')
	
	if i == 'vr':
		interface = vr_interface.IVR(r)
		vr = True
	elif i == 'keyboard':
		interface = keyboard_interface.IKeyboard(r)
		vr = False
	elif i == 'cmd':
		interface = cmd_interface.ICmd(r, s)
		vr = False
	else:
		raise NotImplementedError('Non-supported interface.')
	
	simulator = BulletSimulator(model, interface)

	if j == 'record':
		simulator.setup(repo[t], 0, vr)
		simulator.run(fn, True, v)
	elif j == 'replay':
		# Default view point setting
		simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)
		if os.path.isfile(pjoin(RECORD_LOG_DIR, 'traj.' + fn)):
			simulator.setup(repo[t], 1, vr)
			simulator.playback(fn, d)
		else:
			raise IOError('Record file not found.')
	elif j == 'run':
		simulator.setup(repo[t], 0, vr)
		simulator.run()
	else:
		raise NotImplementedError('Invalid input: Job not recognized.')
	return simulator

def usage():
	print('Please specify at least the interface, model and user job. Default task: ball')
	print('Usage: python node.py -i [interface] -m [model] -j [job] -v <video> -d [delay] -t [task] -r <remote> -s <socket>')

def main(argv):

	interface = None
	model = None
	job = 'play'
	video = False
	delay = 0.0005
	task = 'ball'
	remote = False
	socket = None

	try:
		opts, args = getopt.getopt(argv, 'hi:m:j:vd:t:r:s:', ['help', 
			'interface=', 'model=', 'job=', 'video','delay=', 'task=', 'remote', 'socket='])
	except getopt.GetoptError:
		usage()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			usage()
			sys.exit(0)
		elif opt in ('-i', '--interface'):
			interface = arg
		elif opt in ('-m', '--model'):
			model = arg
		elif opt in ('-j', '--job'):
			job = arg
		elif opt in ('-v', '--video'):
			video = True
		elif opt in ('-d', '--delay'):
			delay = float(arg)
		elif opt in ('-t', '--task'):
			task = arg
		elif opt in ('-r', '--remote'):
			remote = True
		elif opt in ('-s', '--socket'):
			socket = arg
	execute(interface, model, job, video, delay, task, remote, socket)

if __name__ == '__main__':
	main(sys.argv[1:])
