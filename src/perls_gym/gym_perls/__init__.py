from gym.envs.registration import register
import os.path as osp

register(
    id='perls-v0',
    entry_point='perls.src.perls_gym.gym_perls.envs:PerlsEnv',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym.xml'))),
)

register(
    id='exp-v0',
    entry_point='perls.src.perls_gym.gym_perls.envs:PushCube',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../push.xml'))),
)

# import gym
# env = gym.make('exp-v0')
# o = env.reset()

# while True:
#     env.step(None)
