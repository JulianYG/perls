import pybullet as p
from VRcontrol import *
import os, sys, getopt
import json

REPO_DIR = './task.json'

def execute(*args):
	with open(REPO_DIR, 'r') as f:
		repo = json.loads(f.read())
	s, m, v, d, t = args
	if s == 'kuka1':
		simulator = SingleKukaVR(p, repo[t])
	elif s == 'kuka2':
		simulator = DoubleKukaVR(p, repo[t])
	# elif s == 'pr2':
	elif s == 'grasp':
		simulator = DemoVR(p, repo[t])
	elif s == 'pr2':
		simulator = PR2GripperVR(p, repo[t])
	else:
		raise NotImplementedError('Invalid input: Simulator type not recognized.')

	simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

	if m == 'record':
		simulator.record(s, v)
	elif m == 'replay':
		if os.path.isfile('./generic.' + s):
			simulator.replay(s, d)
		else:
			raise IOError('Record file not found.')
	else:
		raise NotImplementedError('Invalid input: Mode not recognized.')

def usage():
	print('Please specify at least the simulator and user mode. Default task: ball')
	print('Usage: python engine.py -s [simulator] -m [mode] -v <video> -d [delay] -t [task]')

def main(argv):
	simulator = None
	mode = 'record'
	video = False
	delay = 0.0005
	task = 'ball'

	try:
		opts, args = getopt.getopt(argv, 'hs:m:vd:t:', ['help', 
			'simulator=', 'mode=', 'video','delay=', 'task='])
	except getopt.GetoptError:
		usage()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			usage()
			sys.exit(0)
		elif opt in ('-s', '--simulator'):
			simulator = arg
		elif opt in ('-m', '--mode'):
			mode = arg
		elif opt in ('-v', '--video'):
			video = True
		elif opt in ('-d', '--delay'):
			delay = float(arg)
		elif opt in ('-t', '--task'):
			task = arg
	execute(simulator, mode, video, delay, task)

if __name__ == '__main__':
	main(sys.argv[1:])


