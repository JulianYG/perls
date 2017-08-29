
import pybullet as p

p.connect(p.DIRECT)

r = p.loadURDF("/Users/ajaymandlekar/Desktop/Dropbox/Stanford/ccr/bullet3/data/sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf",
        (-0.2, -0.7, 0.9), useFixedBase=True)


def fk(joint_pos):
    """
    WARNING: returns eef pose in world frame
    """

    p.setJointMotorControlArray(r, jointIndices=range(7),
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=joint_pos,
                            targetVelocities=(0.,) * 7,
                            velocityGains=(1., ) * 7,
                            positionGains=(.06, ) * 7)
    for _ in range(200):
        p.stepSimulation()

    return p.getLinkState(r, 6)[0], p.getLinkState(r, 6)[1]




