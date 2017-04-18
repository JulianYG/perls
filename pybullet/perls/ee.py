"""
Gym pybullet interface
"""
import gym, bullet
import sys, importlib
import os, getopt, json
import numpy as np
from bullet.models import *
from bullet.simulator import BulletSimulator
from bullet.env.grasp_gym import GraspBulletEnv
from bullet.control import *
from bullet.env.rl import *
from gym.envs.registration import registry, register, make, spec

def execute(*args):

	n, m, r, t, s = args
	module = eval(s)

	TASK_DIR = './data/task.json'
	with open(TASK_DIR, 'r') as f:
		repo = json.loads(f.read())
	num_episodes = 10

	if m == 'kuka':
		# Change Fixed to True for keyboard
		model = kuka.Kuka([0.3], fixed=True, enableForceSensor=False)
	elif m == 'sawyer':
		model = sawyer.Sawyer([0.0], fixed=True, enableForceSensor=False)
	elif m == 'pr2':
		model = pr2.PR2([0.3, -0.5], enableForceSensor=False)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')

	# Simulator is only used for rendering
	# Since simulator is never run, it's ok to just pass None as interface
	simulator = BulletSimulator(model, None)

	# targetPosX, targetPosY, targetPosZ, roll, 
	# pitch, yaw, dist, width=600, height=540
	camera_info = [.8, -.2, 1, 0, -90, 120, 1, 600, 540]
	simulator.set_camera_view(*camera_info)

	register(
	    id='GraspBulletEnv-v0',
	    entry_point='bullet.env.grasp_gym:GraspBulletEnv',
	    timestep_limit=1000,
	    reward_threshold=950.0,
	    kwargs={'simulator': simulator, 'task': repo[t], 
	    		'step_func': module.step_helper, 'realTime': r}
	)

	env = gym.make('GraspBulletEnv-v0')

	weights = module.init_weights()

	for episode in range(n):

		observation = env.reset()
		done = False

		# One horizon
		t = 0
		while not done:
			print(observation)
			# Define action here as well
			action = module.predict(model, weights)
			print(action)
			observation, reward, done, info = env.step(action)
			if done:
				print("Episode finished after {} timesteps".format(t + 1))
            	break
            # t += 1

def main(argv):

	model = None
	num_episodes = 0
	real_sim = False
	task = 'ball'
	step = 'ik'

	try:
		opts, args = getopt.getopt(argv, 'hn:m:rt:j:', ['help', 
			'num_episodes=', 'model=', 'realTimeSimulation','task=', 'step_function='])
	except getopt.GetoptError:
		usage()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			usage()
			sys.exit(0)
		elif opt in ('-n', '--num-episodes'):
			num_episodes = int(arg)
		elif opt in ('-m', '--model'):
			model = arg
		elif opt in ('-r', '--real-time-simulation'):
			real_sim = True
		elif opt in ('-t', '--task'):
			task = arg
		elif opt in ('-s', '--step-function'):
			step = arg

	execute(num_episodes, model, real_sim, task, step)

def usage():
	print('Please specify at least the number of episodes, model, and step method. Default task: ball')
	print('Usage: python ee.py -n [num_episodes] -m [model] -s [step] -r <real-time> -t [task] -r <remote>')

if __name__ == '__main__':
	main(sys.argv[1:])



            	