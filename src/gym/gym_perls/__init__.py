from gym.envs.registration import register
import os.path as osp

register(
    id='perls-v0',
    entry_point='perls.src.gym.gym_perls.envs:PerlsEnv',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-disp.xml'))),
)

register(
    id='push-gui-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCube',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-disp.xml'))),
)

register(
    id='push-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCube',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-cmd.xml'))),
)
