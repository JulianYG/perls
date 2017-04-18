import numpy as np

# Make use of self.model
def step_helper(model, action):	
	"""
	User defined step_helper. Takes pybullet tool model and action,
	returns tuple of observation, reward, done status, and info.
	A high level pybullet wrapper for the gym step function
	""" 
	# Define initial state
	if action == None:
		return np.array([0.3, 0.1, .6]), np.array([0, 1, 0, 0])
	kuka = model.get_tool_ids()[0]

	joint_states = np.array(model.get_tool_joint_states(kuka))
	model.reach(kuka, action[0], action[1], fixed=True)

	return model.get_tool_joint_states(kuka), 1., True, {}

def init_weights():
	# just fit the shape
	# return np.random.random((2, 2))
	return np.random.random((1, 3))

def predict(model, weights):
	# Simple matmul

	# return np.array(model.get_tool_joint_states(kuka)).dot(weights)
	return np.array([-0.2, 0.3, .9]), np.array([0, 1, 0, 0])

