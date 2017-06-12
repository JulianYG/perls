import numpy as np

class Wrapper(object):

	"""
	A base wrapper for reset and stepping in gym
	environment
	"""
	def __init__(self, agent):

		self.agent = agent

	def step(self, action):
		raise NotImplementedError

	def reset(self, scene, task, gui):
		raise NotImplementedError

	@property
	def states(self):

		"""
		State defined as 14 DOF:
		x - y, v, j1-j7, u1-u7
		Delta pos, eef vel, joint pos, joint vel
		"""

		goal = np.array([0.8, 0., 1.])

		arm = self.agent.get_tool_ids()
		assert len(arm) == 1

		arm = arm[0]
		eef_pos = self.agent.get_tool_pose(arm)[0, :]

		delta = eef_pos - goal
		joint_info = self.agent.get_tool_joint_states(arm)[0]

		return (list(delta), self.agent.REST_POSE, list(joint_info[:, 0]),
			list(joint_info[:, 2]))

