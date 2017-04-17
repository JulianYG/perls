

# Make use of self.model
def step_helper(model, action, *args):	
	"""
	User defined step_helper. Takes pybullet tool model and action,
	returns tuple of observation, reward, done status, and info.
	A high level pybullet wrapper for the gym step function
	"""
	kuka = model.get_tool_ids()[0]
	model.reach(kuka, action[0], action[1], fixed=True)

	print(model.get_tool_joint_states(kuka).shape)
	return model.get_tool_joint_states(kuka), 1., True, {}

def init_weights():
	return

def predict(model, weights):

	# print(weights.shape)
	pass
