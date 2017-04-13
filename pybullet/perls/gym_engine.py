"""
Can either interface with pybullet or 
"""
import gym
import numpy as np
from bullet.env.grasp_gym import GraspBulletEnv

def reward():
	# Make use of self.model
	pass

def observation(dim):
	# Make use of self.model
	pass

def step():	
	# Make use of self.model
	pass

def main():
	# Example initialization of feature dimensions
	feat_dim = (20, 10)
	weights = np.random.random(feat_dimension)
	model = Kuka([-0.1], fixed=True)
	agent = bullet.simulator.BulletSimulator(model, 
		bullet.control.cmd_interface.ICmd(hub))
	#model = PR2([-0.5, 0.3])
	env = gym.make(
		'GraspBulletEnv-v0', 		# Environment
		agent, 						# simulator
		task, 						# Task
		reward,						# Reward function
		observation(feat_dim),		# Observe function
		step						# Self-defined step function
	)

	for _ in range(1):
		observation = env.reset()
		done = False
		t = 0
		while not done:
			print(observation)
			# Define action here as well
			action = np.array([np.dot(observation, weights)])
			print(action)
			observation, reward, done, info = env.step(action)
			t += 1
			if done:
				print("Episode finished after {} timesteps".format(t + 1))
            	break


if __name__ == 'main':
	main()



            	