from gym.envs.registration import register
import os.path as osp


register(
    id='perls-v0',
    entry_point='gym_perls.envs:PerlsEnv',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym.xml'))),
)

register(
    id='exp-v0',
    entry_point='gym_perls.envs:PushCube',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym.xml'))),
)

