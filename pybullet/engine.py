import pybullet as p
from VRcontrol import *
# from render import generate_trajectory

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

kukaSimulator = KukaDoubleArmVR(p, repo['ball'])
kukaSimulator.set_camera_view(-.4, -.2, 1, 0, -90, 120, 1)

kukaSimulator.record('try', saveVideo=0)
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
