import gym
import numpy as np
from bullet.env.robot_gym import RobotBulletEnv

w = [0.3, 0.02, 0.02, 0.012]

if __main__():
	final_pos = [0.4, 1.5, 0.8]
	env = gym.make('RobotBulletEnv-v0', Kuka([-0.1], fixed=True), final_pos)

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



            	