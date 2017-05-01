import numpy as np

# Make use of self.agent
def step_helper(agent, action):	
	"""
	User defined step_helper. Takes pybullet tool agent and action,
	returns tuple of observation, reward, done status, and info.
	A high level pybullet wrapper for the gym step function
	""" 
	# Define initial state
	kuka = agent.get_tool_ids()[0]
	if action == None:
		return ([0.8, 0., 1.], (0, 1, 0, 0)), 0., False, {}

	joint_states = agent.get_tool_joint_states(kuka)[0]
	agent.reach(kuka, action[0], action[1], fixed=True, null_space=False, expedite=True)
	
	reached = False
	eef_pos = agent.get_tool_pose(kuka)[0]

	if np.sqrt(np.sum((np.array(action[0])[:2] - np.array(eef_pos)[:2]) ** 2)) < 1e-6 and\
		abs(action[0][2] - eef_pos[2]) < 0.025:
		reached = True

	return eef_pos, 1., reached, {}

# def init_weights():
# 	# just fit the shape
# 	# return np.random.random((2, 2))
# 	return np.random.random((1, 3))

# def predict(agent, weights):
# 	# Simple matmul
# 	pos = []
# 	# return np.array(agent.get_tool_joint_states(kuka)).dot(weights)
# 	return np.array([0.8, 0., 1.]), np.array([0, 1, 0, 0])

