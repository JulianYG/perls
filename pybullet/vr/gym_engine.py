import gym
import numpy as np
from bullet.env.kuka_bullet import KukaBulletEnv

w = [0.3, 0.02, 0.02, 0.012]

if __main__():

	env = gym.make('KukaBulletEnv-v0')

	for _ in range(1):
		observation = env.reset()
		done = False
		t = 0
		while not done:
			print(observation)
			action = np.array([np.inner(observation, w)])
			print(action)
			observation, reward, done, info = env.step(action)
			t += 1
			if done:
				print("Episode finished after {} timesteps".format(t+1))
            	break



            	