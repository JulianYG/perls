import pybullet as p
from rl import BulletPhysicsVR
from render import generate_trajectory

generate_trajectory((6,6,6), (0,0,0), 'see.csv', 'newsee.csv')
simulator = BulletPhysicsVR(p, 2)

simulator.replay('see.csv')

# simulator.replay('newsee.csv')
