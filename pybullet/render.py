import csv
from pybullet import *
import numpy as np
import math

def generate_trajectory(camera_pos, focus_pos, filename, new_file):
	f = open(filename, 'r')
	r_f = open(new_file, 'w', newline='')
	reader = csv.reader(f)
	writer = csv.writer(r_f)
	for row in reader:
		if float(row[0]) == -1:
			writer.writerow(row)
		else:
			ob = row[0]
			pos = row[1:4]
			orien = row[4:]
			transformed_pose = [ob] + transform(pos, orien, camera_pos, focus_pos)
			writer.writerow(transformed_pose)
	f.close()
	r_f.close()

def transform(position, orientation, camera_pos, focus_pos):

	viewMatrix = np.array(computeViewMatrix(camera_pos, focus_pos, (0,0,1))).reshape((4, 4))
	rotationMatrix = quat2mat(*orientation)
	upper = np.column_stack((rotationMatrix, list(position)))
	poseMatrix = np.array(np.concatenate((upper, [[0,0,0,1]]), 0), dtype='float64')
	# print(poseMatrix)
	cameraMatrixInv = np.linalg.inv(viewMatrix)
	# print(cameraMatrixInv)
	cameraView = np.dot(poseMatrix, cameraMatrixInv)

	orien = getQuaternionFromEuler(mat2quat(cameraView[:3, :3]))
	pos = cameraView[:3, 3]
	return [pos[0], pos[1], pos[2], orien[0], orien[1], orien[2], orien[3]]

def quat2mat(w, x, y, z):

	theta = getEulerFromQuaternion([float(w), float(x), float(y), float(z)])
	
	R_x = np.array([[1, 0, 0], [0, math.cos(theta[0]), -math.sin(theta[0])], 
		[0, math.sin(theta[0]), math.cos(theta[0])]])

	R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])], [0, 1, 0], 
		[-math.sin(theta[1]), 0, math.cos(theta[1])]])

	R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0], 
		[math.sin(theta[2]), math.cos(theta[2]), 0],  [0, 0, 1]])
	R = np.dot(R_z, np.dot(R_y, R_x))
	return R

def mat2quat(R):

	sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

	singular = sy < 1e-6
	if not singular:
		x = math.atan2(R[2,1] , R[2,2])
		y = math.atan2(-R[2,0], sy)
		z = math.atan2(R[1,0], R[0,0])
	else:
		x = math.atan2(-R[1,2], R[1,1])
		y = math.atan2(-R[2,0], sy)
		z = 0
	return (x, y, z)



# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6