
import pybullet as p

p.connect(p.DIRECT)

r = p.loadURDF('/Users/JulianYGao/bullet3/data/sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf',
           position=(-0.2, -0.7, 0.9), useFixedBase=True)


def fk(joint_pos):

    p.setJointMotorControlArray(r, jointIndices=range(7),
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=joint_pos,
                            targetVelocities=(0.,) * 7,
                            velocityGains=(1., ) * 7,
                            positionGains=(.06, ) * 7)
    for _ in range(200):
        p.stepSimulation()

    return p.getLinkState(r, 6)[0]




