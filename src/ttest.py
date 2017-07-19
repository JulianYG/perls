import pybullet as p
# import time
# import math
# from datetime import datetime
# import thread

from lib.control import Controller as sc

s = sc('config.xml')
# s.start_all()
s.start()


######
# from lib.state.physicsEngine import BulletPhysicsEngine as bpe
# # from lib.render.renderEngine import BulletRenderEngine as bre
# b = bpe(0, 0, 0, async=True)

# # r = bre('')

# p.connect(p.DIRECT)


# p.resetSimulation()

# from lib.entity.sawyer import Sawyer

# s = Sawyer('m0', b)

# while True:
# 	b.step(0)
# class X():
#     def __init__(self, name):
#         self.name = name
#     def __str__(self):
#         return self.name

# x = X('ha')
# x
# print(x)
# s = p.connect(p.DIRECT)
# p.resetSimulation(s)
# p.setRealTimeSimulation(0, s)


# c = p.loadURDF('cube_small.urdf', physicsClientId=s)

# p.setGravity(0,0,10., s)


# def haha(_):
# 	q = p.connect(p.SHARED_MEMORY, 2)
# 	p.setInternalSimFlags(0, q)
# 	p.resetSimulation(q)
# 	p.setRealTimeSimulation(0, q)

# 	c = p.loadURDF('cube_small.urdf', physicsClientId=q)

# 	p.setGravity(0,0,10., q)
# 	while True:
# 		p.stepSimulation(q)
# 		# print(p.getBasePositionAndOrientation(c, physicsClientId=q)[0])

# thread.start_new_thread(haha, (None,))

# while True:
# 	p.stepSimulation(s)
	# print(p.getBasePositionAndOrientation(c, physicsClientId=s)[0])


# #clid = p.connect(p.SHARED_MEMORY)
# p.connect(p.GUI)
# p.loadURDF("plane.urdf",[0,0,-0.3])
# kukaId = p.loadURDF("kuka_iiwa/model_vr_limits.urdf",[0,0,0])
# p.resetBasePositionAndOrientation(kukaId,[0,0,0],[0,0,0,1])
# kukaEndEffectorIndex = 6
# numJoints = p.getNumJoints(kukaId)
# if (numJoints!=7):
# 	exit()
	
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
# #lower limits for null space
# ll=[-.967,-2	,-2.96,0.19,-2.96,-2.09,-3.05]
# #upper limits for null space
# ul=[.967,2	,2.96,2.29,2.96,2.09,3.05]
# #joint ranges for null space
# jr=[5.8,4,5.8,4,5.8,4,6]
# #restposes for null space
# rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
# #joint damping coefficents
# jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]

# for i in range (numJoints):
# 	p.resetJointState(kukaId,i,rp[i])

# p.setGravity(0,0,0)

# p.setRealTimeSimulation(1)
# #trailDuration is duration (in seconds) after debug lines will be removed automatically
# #use 0 for no-removal

	
# while 1:

# 	for i in range (100):
# 		pos = [-0.6, 0., 0.2]
# 		#end effector points down, not up (in case useOrientation==1)
# 		orn = (0,1,0,0)
# 		jointPoses = p.calculateInverseKinematics(kukaId,kukaEndEffectorIndex,pos,orn,ll,ul,jr,rp,jointDamping=jd)
		
# 		for i in range (numJoints):
# 			p.setJointMotorControl2(bodyIndex=kukaId,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)

# 	print(p.getLinkState(kukaId, 6)[0])

