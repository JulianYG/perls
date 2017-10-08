#!/usr/bin/env python

from gym.envs.registration import register
import os.path as osp


register(
    id='push-vel-gui-v0',
    entry_point='perls.gym_.gym_perls.envs:PushCubeVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-disp.xml')),
                max_step=1000),
)

register(
    id='push-vel-v0',
    entry_point='perls.gym_.gym_perls.envs:PushCubeVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-cmd.xml')),
                max_step=1000),
)

register(
    id='push-pose-gui-v0',
    entry_point='perls.gym_.gym_perls.envs:PushCubePose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-disp.xml')),
                max_step=1000),
)

register(
    id='push-pose-v0',
    entry_point='perls.gym_.gym_perls.envs:PushCubePose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-cmd.xml')),
                max_step=1000),
)

register(
    id='push-viz-vel-v0',
    entry_point='perls.gym_.gym_perls.envs:PushVizVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-cmd.xml')),
                max_step=1000),
)

register(
    id='push-viz-pose-v0',
    entry_point='perls.gym_.gym_perls.envs:PushVizPose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-cmd.xml')),
                max_step=1000),
)

register(
    id='push-viz-pose-gui-v0',
    entry_point='perls.gym_.gym_perls.envs:PushVizPose',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-disp.xml')),
                max_step=1000),
)

register(
    id='push-viz-vel-gui-v0',
    entry_point='perls.gym_.gym_perls.envs:PushVizVel',
    kwargs=dict(conf_path=osp.abspath(osp.join(__file__, '../../../configs/gym-disp.xml')),
                max_step=1000),
)

register(
    id='push-robot-v0',
    entry_point='perls.gym_.gym_perls.envs:PushRobot',
)

