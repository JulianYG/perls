import pybullet as p
import multiprocessing

jobs = []

def work(n):

    p.connect(p.DIRECT)
    p.setRealTimeSimulation(1)
    p.loadURDF('cube_small.urdf')
    p.setGravity(0, 0, -9.81)
    try:
        while 1:
            p.stepSimulation()
            print(p.getBasePositionAndOrientation(0), n)
    except KeyboardInterrupt:
        

for i in range(3):
    x = multiprocessing.Process(target=work, args=(i,))
    jobs.append(x)
    print(dir(x))
    x.start()


# from lib.utils import math_util
# p.connect(p.GUI)

# # p.setRealTimeSimulation(1)
    
# import numpy as np
# np.set_printoptions(2, suppress=True)

# width, height, vmat, projmat, up, forward, _, _, yaw, pitch, dist, target = p.getDebugVisualizerCamera()

# vmat = np.array(vmat).reshape((4,4))

# print (vmat, 'view matrix')

# perm = [2, 0, 1]

# if up == (0, 1, 0):
#     print ('multiplied'  * 20)
#     vmat = vmat.dot(np.array(
#         [[-1, 0, 0, 0],
#          [0, 0, 1, 0],
#          [0, 1, 0, 0],
#          [0, 0, 0, 1]],
#         dtype=np.float32
#     ))

# transform_mat = np.linalg.inv(vmat)

# projmat = np.array(projmat).reshape((4,4))
# vp = projmat.dot(vmat)

# # print(up, forward)
# # print(proj[:3,:3].dot(x[:3,:3]).dot(np.array([512,378,1])))
# # point3d = vp.dot(np.array([0,0,0,1]))

# # print(width, height)
# # print((point3d[0] + 1) / 2. * width /2 , (1 - point3d[1]) / 2. * height/2)
# p.loadURDF('cube_small.urdf',(1,0,0), useFixedBase=False)
# print(transform_mat, "Transformation matrix")

# # print(p.getDebugVisualizerCamera())
# _AXES2TUPLE = {
#     'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
#     'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
#     'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
#     'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
#     'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
#     'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
#     'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
#     'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

# for x in _AXES2TUPLE.keys():
#     print(math_util.mat2euler(transform_mat[:3,:3], axes=x) * 180 / np.pi, x)

# while 1:
#     p.stepSimulation()
#     for e in p.getMouseEvents():
#         if e[0] == 2:
#             # print(e[1], e[2], 0)
#             x = 2. * e[1] / width - 1
#             y = -2. * e[2] / height + 1
#             print(np.linalg.inv(vp)[:3,:3].dot(np.array([x,y,0])))
