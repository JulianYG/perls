"""
Gym pybullet interface
"""
import gym
import numpy as np
from bullet.env.grasp_gym import GraspBulletEnv
from bullet.control.hub import *
from bullet.env import rl

TASK_DIR = './data/task.json'

def main():

	weights = rl.ik.init_weights()
	num_episodes = 10

	model = Kuka([-0.1], fixed=True)
	#model = PR2([-0.5, 0.3])

	# Simulator is only used for rendering
	simulator = bullet.simulator.BulletSimulator(model, 
		singleton_interface.ISingleton())

	env = gym.make(
		'GraspBulletEnv-v0', 		# Environment
		simulator, 					# pybullet simulator
		task, 						# Task
		rl.ik.step,					# Self-defined step function
		False						# Not using real-time simulation
	)

	for episode in range(num_episodes):
		observation = env.reset()
		done = False

		# One horizon
		while not done:
			print(observation)
			# Define action here as well
			action = rl.ik.predict(model, weights)
			print(action)
			observation, reward, done, info = env.step(action)
			if done:
				print("Episode finished after {} timesteps".format(t + 1))
            	break
            	
            # targetPosX, targetPosY, targetPosZ, roll, 
			# pitch, yaw, dist, width=600, height=540

            # env.render(.8, -.2, 1, 0, -90, 120, 1, 600, 540, mode='rgb_array')

if __name__ == 'main':
	main()



            	