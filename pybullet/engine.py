import pybullet as p
from rl import BulletPhysicsVR
from render import generate_trajectory



# generate_trajectory((6,6,6), (0,0,0), 'see.csv', 'newsee.csv')
simulator = BulletPhysicsVR(p, 2)
1.4,-0.2,0.6
simulator.set_camera_view(.8, -.2, 1, 0, -90, 120, 1)
# simulator.replay('see.csv', 1)
simulator.replay('see.csv')
# simulator.replay('newsee.csv')
