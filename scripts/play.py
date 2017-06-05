import gym, sys
import os
from os.path import join as pjoin
path = os.path.split(os.path.abspath(os.getcwd()))
rpath = '/'.join(path[: path.index('perls') + 1])
sys.path.append(pjoin(rpath, 'src/gym_'))

import numpy as np
from gym_bullet.envs.bullet_env import BulletEnv

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
			
			# Toy example: IK
			# Position Control
			# action = ([0.8, 0, 1.0], (0, 1, 0, 0))

			# Torque control
			action = np.array([900, 200, 300, 400, 500, 600, 100], dtype=np.float32)

			# Velocity control
			# action = [np.array([0.2, -0.02, 0., -0.5, 0.1, 0., 0.3], dtype=np.float32)]

			print('action ****')
			print(action)
			# break
			observation, reward, done, info = env.step(action)
			
			task += 1
			if done:
				print("Episode finished after {} timesteps".format(task + 1))
				env.render(mode='segment')
				break


if __name__ == '__main__':
	execute(sys.argv[1:])

				

