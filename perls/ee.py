"""
Gym pybullet interface
"""
import gym, bullet
import sys, importlib
import os, getopt, json
import numpy as np
from bullet.agents import *
from bullet.simulator import BulletSimulator
from bullet.env.grasp_gym import GraspBulletEnv
from bullet.control import *
from bullet.env.rl import *
from gym.envs.registration import registry, register, make, spec
import bullet.util as utils
from os.path import join as pjoin

def execute(*args):

	env = gym.make('BulletEnv-v0')

	weights = module.init_weights()

	for episode in range(num_episodes):

		observation = env.reset()

		# for _ in xrange(100):
		# 	simulator.step_simulation()

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
            # task += 1

def usage():
	print('Please specify the configuration file under ./configs. Default config: 1')
	print('Usage: python node.py -c [config]')

def main(argv):

	config = '1'
	try:
		opts, args = getopt.getopt(argv, 'hc:', ['help', 'config='])
	except getopt.GetoptError:
		usage()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			usage()
			sys.exit(0)
		elif opt in ('-c', '--config'):
			config = arg
	execute(config)

if __name__ == '__main__':
	main(sys.argv[1:])

            	