import gym, sys
from gym_bullet.envs.bullet_env import BulletEnv
import numpy as np
from gym_bullet.wrappers import ik as module

def execute(*args):

	env = gym.make('bullet-v0')

	weights = module.init_weights()

	for episode in range(10000):

		observation = env.reset()
		done = False

		# One horizon
		task = 0
		while not done:
			print('observation ****')
			print(observation)
			# Define action here as well
			action = module.predict(agent, weights)
			# action = ([observation[0][0], observation[0][1],
			# 	observation[0][2] + 0.1], observation[1])
			print('action ****')
			print(action)
			# break
			observation, reward, done, info = env.step(action)
			task += 1
			if done:
				print("Episode finished after {} timesteps".format(task + 1))
            	break

if __name__ == '__main__':
	execute(sys.argv[1:])

            	

