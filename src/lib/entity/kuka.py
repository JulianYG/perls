from .arm import Arm
from ..utils import math_util


class Kuka(Arm):

    def __init__(self, tool_id, engine,
                 path=None,
                 pos=(0., 0., 0.67),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or 'kuka_iiwa/model_vr_limits.urdf'
        super(Kuka, self).__init__(
            tool_id, engine, path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.045])
        self._rest_pose = (0., 0., 0., 1.570793, 0., -1.04719755, 0.)

        self.reset()

    @property
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        return range(7)
