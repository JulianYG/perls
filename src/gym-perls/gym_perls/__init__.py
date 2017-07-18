from gym.envs.registration import register
import os.path as osp

register(
    id='perls-v0',
    entry_point='envs:PerlsEnv',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym.xml'))),
)

register(
    id='exp-v0',
    entry_point='envs:PushCube',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym.xml'))),
)

import gym
env = gym.make('exp-v0')
o = env.reset()
print(o)
import pybullet as p
while True:
    p.stepSimulation()
