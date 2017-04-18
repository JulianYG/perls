import numpy as np
import pybullet as p
# Make use of self.model
def step_helper(model, action):	
	"""
	User defined step_helper. Takes pybullet tool model and action,
	returns tuple of observation, reward, done status, and info.
	A high level pybullet wrapper for the gym step function
	""" 
	# Define initial state
	kuka = model.get_tool_ids()[0]
	if action == None:
		# b = p.loadURDF("sphere_small.urdf", p.getBasePositionAndOrientation(model.grippers[0])[0], 
		# 	(0, 0, 0, 1))
		# p.createConstraint(b, -1, -1, -1, p.JOINT_FIXED, [0, 0,0], [0,0,0],[0,0,1])
		# c = p.loadURDF("sphere_small.urdf", (0.86, 0, 1.1164), 
		# 	(0, 0, 0, 1))
		# p.createConstraint(c, -1, -1, -1, p.JOINT_FIXED, [0, 0,0], [0,0,0],[0,0,1])
		return ([0.8, 0.3, 1.2], (0, 1, 0, 0)), 1., False, {}

	joint_states = np.array(model.get_tool_joint_states(kuka))
	model.reach(kuka, action[0], action[1], fixed=True)

	# reached = True

	return model.get_tool_pose(kuka), 1., False, {}

def init_weights():
	# just fit the shape
	# return np.random.random((2, 2))
	return np.random.random((1, 3))

def predict(model, weights):
	# Simple matmul
	pos = []
	# return np.array(model.get_tool_joint_states(kuka)).dot(weights)
	return np.array([0.8, 0.3, 1.2]), np.array([0, 1, 0, 0])

