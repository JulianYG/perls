import pybullet as p

p.connect(p.GUI)

p.setRealTimeSimulation(1)
	
import numpy as np

width, height, vmat, projmat, _, _, _, _, yaw, pitch, dist, target = p.getDebugVisualizerCamera()

vmat = np.array(vmat).reshape((4,4))
projmat = np.array(projmat).reshape((4,4))
vp = projmat.dot(vmat)

# print(proj[:3,:3].dot(x[:3,:3]).dot(np.array([512,378,1])))
point3d = vp.dot(np.array([0,0,0,1]))

# print(width, height)
print((point3d[0] + 1) / 2. * width /2 , (1 - point3d[1]) / 2. * height/2)
p.loadURDF('cube_small.urdf',(1,0,0))
while 1:

	for e in p.getMouseEvents():
		if e[0] == 2:
			# print(e[1], e[2], 0)
			x = 2. * e[1] * 2. / width- 1
			y = -2. * e[2] * 2. / height + 1
			print(np.linalg.inv(vp)[:3,:3].dot(np.array([x,y,0])))