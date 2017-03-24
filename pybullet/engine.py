import pybullet as p
from VRcontrol import *
import os, sys, getopt

repo = {}
# Indicate the indices of the objects that need to be tracked in the first entry
repo['hanoi'] = [("cvgl/pole.urdf",-0.80000,0.100000,0.699990,0.00000,0.0,0.00000,1), 
	("cvgl/pole.urdf",-0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1),
	("cvgl/pole.urdf",-0.80000,-0.500000,0.699990,0.00000,0.0,0.00000,1),
	("cvgl/torus_0.urdf",-0.81,0.1,0.69999,1,0,0,1),
	("cvgl/torus_1.urdf",-0.82,0.1,0.69999,1,0,0,1),
	("cvgl/torus_2.urdf",-0.83,0.1,0.69999,1,0,0,1),
	("cvgl/torus_3.urdf",-0.84,0.1,0.69999,1,0,0,1),
	("cvgl/torus_4.urdf",-0.88,0.1,0.69999,0,0,0,1)]

repo['ball'] = [("sphere_small.urdf",0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.94000,-0.1400000,0.729990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.83000,-0.520000,0.699990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.92000,-0.1400000,0.729990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.83000,-0.520000,0.699990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.96000,-0.1400000,0.729990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.87000,-0.520000,0.699990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.91000,-0.1400000,0.729990,0.000000,0.0,0.00000,1),
		("sphere_small.urdf",0.82000,-0.520000,0.699990,0.000000,0.0,0.00000,1)]
		# ,
		# ("tray/tray_textured2.urdf", 0.94, -0.11, 0.6, 0, 0, 0, 1)]

def execute(s, m):

	if s == 'kuka1':
		simulator = KukaSingleArmVR(p, repo['ball'])
	elif s == 'kuka2':
		simulator = KukaDoubleArmVR(p, repo['ball'])
	# elif s == 'pr2':
	elif s == 'grasp':
		simulator = DemoVR(p, repo['ball'])
	else:
		raise NotImplementedError('Invalid input: Simulator type not recognized.')

	simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

	if m == 'record':
		simulator.record(s)
	elif m == 'replay':
		if os.path.isfile('./generic.' + s):
			simulator.replay(s)
		else:
			raise IOError('Record file not found.')
	else:
		raise NotImplementedError('Invalid input: Mode not recognized.')

def usage():
	print('Please specify the simulator and user mode')
	print('Usage: python engine.py -s <simulator> -m <mode>')

def main(argv):
	simulator = 'double'
	mode = 'record'
	try:
		opts, args = getopt.getopt(argv, 'hs:m:', ['help', 'simulator=', 'mode='])
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
	execute(simulator, mode)

if __name__ == '__main__':
	main(sys.argv[1:])

# kukaSimulator = KukaDoubleArmVR(p, repo['ball'])
# kukaSimulator.set_camera_view(-.4, -.2, 1, 0, -90, 120, 1)

# kukaSimulator.record('try', saveVideo=0)
# kukaSimulator.replay('try')

# graspSimulator = PR2GripperVR(p, repo['ball'])
# graspSimulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

# graspSimulator.record('pr2')   

# graspSimulator.replay('pr2')

# demoSimulator = DemoVR(p, repo['ball'])
# demoSimulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

# demoSimulator.record('demo')
# demoSimulator.replay('demo')

# simSimulator = KukaSingleArmVR(p, repo['ball'])
# simSimulator.set_camera_view(-.4, -.2, 1, 0, -90, 120, 1)

# simSimulator.record('one')
# simSimulator.replay('one')
