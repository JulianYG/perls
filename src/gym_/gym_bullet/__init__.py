import os, json, sys
from os.path import join as pjoin
from .wrappers import *
from gym.envs.registration import register

path = os.path.dirname(os.path.abspath(__file__))
rpath = os.path.normpath(pjoin(path, '../../..'))
sys.path.append(pjoin(rpath, 'src'))

from bullet_ import simulation
__package__ = 'bullet_.simulation'

from .utils import io, build_util

bullet_path = pjoin(rpath, 'src/bullet_')

CONFIG_DIR = pjoin(rpath, 'src/gym_/config.json')
_CONFIGS = io.read_config(CONFIG_DIR)

# Simulator is only used for rendering
# Since simulator is never run, it's ok to just pass None as interface
simulator = build_util.build_by_config(_CONFIGS, bullet_path)

num_episodes = _CONFIGS['num_episodes']

record = _CONFIGS['record']
video = _CONFIGS['video']
wrapper = _CONFIGS['wrapper']
time_step = _CONFIGS['time_step']
real_time = _CONFIGS['real_time'] 
step_limit = _CONFIGS['step_limit']
reward_thresh = _CONFIGS['reward_thresh']

if sys.version[0] == '2':
    module = reduce(getattr, wrapper.split("."), sys.modules[__name__])
else:
    import functools
    module = functools.reduce(getattr, wrapper.split("."), sys.modules[__name__])

register(
	id='bullet-v0',
	entry_point='gym_bullet.envs:BulletEnv',
	timestep_limit=step_limit,
	reward_threshold=reward_thresh,
	kwargs={'simulator': simulator, 'wrapper': module(simulator.tool),
			'realTime': real_time, 'time_step': time_step, 
			'record': record, 'video': video}
	)


