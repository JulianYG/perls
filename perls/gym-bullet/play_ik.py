import gym, sys
from gym_bullet.envs.bullet_env import BulletEnv
import numpy as np
from gym_bullet.wrappers import ik as module

def execute(*args):

	env = gym.make('bullet-v0')

	for episode in range(10000):

		observation = env.reset()
		done = False

		# One horizon
		task = 0
		while not done:
			print('observation ****')
			print(observation)
			
			# Define action here as well
			action = ([0.8, 0, 1.0], (0, 1, 0, 0))
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

            	

