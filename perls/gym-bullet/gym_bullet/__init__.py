import os, json, sys
from os.path import join as pjoin
from gym_bullet.wrappers import *
from gym.envs.registration import register

bullet_path = pjoin(os.getcwd(), '../')
sys.path.append(bullet_path)

from bullet.utils import helpers as utils
from bullet.agents import *
from bullet.simulator import BulletSimulator
import bullet.utils.helpers as utils


TASK_DIR = pjoin(bullet_path, 'data', 'task.json')
SCENE_DIR = pjoin(bullet_path, 'data', 'scene.json')

with open(TASK_DIR, 'r') as f:
	task_repo = json.loads(f.read())
with open(SCENE_DIR, 'r') as f:
	scene_repo = json.loads(f.read())

CONFIG_DIR = pjoin(os.getcwd(), 'config.json')
_CONFIGS = utils.read_config(CONFIG_DIR)

num_episodes = _CONFIGS['num_episodes']
agent = _CONFIGS['agent'] 
real_time = _CONFIGS['real_time'] 
task = _CONFIGS['task'] 
step_func = _CONFIGS['step_function']
fixed = _CONFIGS['fixed_gripper_orn']
force_sensor = _CONFIGS['enable_force_sensor']
init_pos = _CONFIGS['tool_positions']
camera_info = _CONFIGS['camera']
time_step = _CONFIGS['time_step']
gui = _CONFIGS['gui']
scene = _CONFIGS['scene']
step_limit = _CONFIGS['step_limit']
reward_thresh = _CONFIGS['reward_thresh']

module = eval(step_func)
print(module, 'a' * 10)
if agent == 'kuka':
	# Change Fixed to True for keyboard
	agent = kuka.Kuka(init_pos, fixed=fixed, enableForceSensor=force_sensor)
elif agent == 'sawyer':
	agent = sawyer.Sawyer(init_pos, fixed=fixed, enableForceSensor=force_sensor)
elif agent == 'pr2':
	agent = pr2.PR2(init_pos, enableForceSensor=force_sensor)
else:
	raise NotImplementedError('Invalid input: Model not recognized.')

# Simulator is only used for rendering
# Since simulator is never run, it's ok to just pass None as interface
simulator = BulletSimulator(agent, None, task_repo[task], scene_repo[scene], gui=gui)
simulator.set_camera_view(*camera_info)

register(
	id='bullet-v0',
	entry_point='gym_bullet.envs:BulletEnv',
	timestep_limit=step_limit,
	reward_threshold=reward_thresh,
	kwargs={'simulator': simulator, 'step_func': module.step_helper, 
			'realTime': real_time, 'time_step': time_step}
)


