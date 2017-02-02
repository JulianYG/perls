import pybullet as p
from rl import BulletPhysicsVR
# from render import generate_trajectory



# generate_trajectory((6,6,6), (0,0,0), 'see.csv', 'newsee.csv')
simulator = BulletPhysicsVR(p, [2, 4], task=1, hand=False)

simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)

simulator.replay('two_arms.csv', saveVideo=0)
# simulator.record('two_arms_hanoi.csv')
