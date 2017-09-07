# !/usr/bin/env python

from .push_viz import PushViz
from lib.utils import math_util


class PushVizPose(PushViz):
    """
    Pushing cube across the table
    """
    def __init__(self, conf_path):

        super(PushVizPose, self).__init__(conf_path)

    @property
    def observation_space(self):
        return NotImplemented

    @property
    def action_space(self):
        return PushCube.Space.Box(
            low=-math_util.vec((0.05, 0.05, 0.05)),
            high=math_util.vec((0.05, 0.05, 0.05))
        )

    @property
    def state(self):

        img = super(PushVizPose, self).state
        goal_pos = self._world.get_task_state()['goal']
        eef_pos, _ = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose)
        aux = math_util.concat(eef_pos, goal_pos)
        return img, aux

    def _step_helper(self, action):

        eef_bframe_pos, eef_bframe_orn = math_util.get_relative_pose(
            self._robot.eef_pose, self._robot.pose
        )

        eef_bframe_pos += math_util.vec(action)
        eef_wframe_pose = math_util.get_absolute_pose(
            (eef_bframe_pos, eef_bframe_orn), 
            self._robot.pose
        )

        self._robot.set_eef_pose(
            eef_wframe_pose[0],
            None, iters=-1
        )
