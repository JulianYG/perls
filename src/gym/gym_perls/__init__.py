from gym.envs.registration import register
import os.path as osp

register(
    id='perls-v0',
    entry_point='perls.src.gym.gym_perls.envs:PerlsEnv',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-disp.xml'))),
)

register(
    id='push-vel-gui-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCubeVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-disp.xml'))),
)

register(
    id='push-vel-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCubeVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-cmd.xml'))),
)

register(
    id='push-pose-gui-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCubePose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-disp.xml'))),
)

register(
    id='push-pose-v0',
    entry_point='perls.src.gym.gym_perls.envs:PushCubePose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../gym-cmd.xml'))),
)

