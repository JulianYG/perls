from bullet.models import *
from bullet.control import *
from bullet.simulator import BulletSimulator
import os, sys, getopt
from os.path import join as pjoin
import json

REPO_DIR = './data/task.json'
RECORD_LOG_DIR = './data/record'

def execute(*args):
	with open(REPO_DIR, 'r') as f:
		repo = json.loads(f.read())
	i, m, j, v, d, t, r = args
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
	else:
		raise NotImplementedError('Non-supported interface.')
	
	simulator = BulletSimulator(model, interface)

	# Default view point setting
	simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

	if j == 'record':
		simulator.setup(repo[t], 0, vr)
		simulator.record(fn, v)
	elif j == 'replay':
		if os.path.isfile(pjoin(RECORD_LOG_DIR, 'generic.' + fn)):
			simulator.setup(repo[t], 1, vr)
			simulator.replay(fn, d)
		else:
			raise IOError('Record file not found.')
	else:
		raise NotImplementedError('Invalid input: Job not recognized.')

def usage():
	print('Please specify at least the interface, model and user job. Default task: ball')
	print('Usage: python ccr_engine.py -i [interface] -m [model] -j [job] -v <video> -d [delay] -t [task] -r <remote>')

def main(argv):
	interface = None
	model = None
	job = 'replay'
	video = False
	delay = 0.0005
	task = 'ball'
	remote = False

	try:
		opts, args = getopt.getopt(argv, 'hi:m:j:vd:t:r', ['help', 
			'interface=', 'model=', 'job=', 'video','delay=', 'task=', 'remote'])
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
	execute(interface, model, job, video, delay, task, remote)

if __name__ == '__main__':
	main(sys.argv[1:])


