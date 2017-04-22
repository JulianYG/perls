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
import bullet.util as utils
from os.path import join as pjoin

def execute(*args):

	REPO_DIR = pjoin(os.getcwd(), 'data', 'task.json')
	with open(REPO_DIR, 'r') as f:
			repo = json.loads(f.read())

	CONFIG_DIR = pjoin(os.getcwd(), 'configs', args[0] + '.json')
	_CONFIGS = utils.read_config(CONFIG_DIR)

	num_episodes = _CONFIGS['num_episodes']
	model = _CONFIGS['model'] 
	real_time = _CONFIGS['real_time'] 
	task = _CONFIGS['task'] 
	step_func = _CONFIGS['step_function']
	fixed = _CONFIGS['fixed_gripper_orn']
	force_sensor = _CONFIGS['enable_force_sensor']
	init_pos = _CONFIGS['tool_positions']
	camera_info = _CONFIGS['camera']
	time_step = _CONFIGS['time_step']

	module = eval(step_func)

	if model == 'kuka':
		# Change Fixed to True for keyboard
		model = kuka.Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif model == 'sawyer':
		model = sawyer.Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
	elif model == 'pr2':
		model = pr2.PR2(init_pos, enableForceSensor=force_sensor)
	else:
		raise NotImplementedError('Invalid input: Model not recognized.')

	# Simulator is only used for rendering
	# Since simulator is never run, it's ok to just pass None as interface
	simulator = BulletSimulator(model, None, repo[task])
	simulator.set_camera_view(*camera_info)

	register(
	    id='GraspBulletEnv-v0',
	    entry_point='bullet.env.grasp_gym:GraspBulletEnv',
	    # timestep_limit=1000,
	    reward_threshold=950.0,
	    kwargs={'simulator': simulator, 'step_func': module.step_helper, 
	    		'realTime': real_time, 'time_step': time_step}
	)

	env = gym.make('GraspBulletEnv-v0')

	weights = module.init_weights()

	for episode in range(num_episodes):

		observation = env.reset()

		for _ in xrange(100):
			simulator.step_simulation()

		done = False

		# One horizon
		task = 0
		while not done:
			print('observation ****')
			print(observation)
			# Define action here as well
			action = module.predict(model, weights)
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

            	