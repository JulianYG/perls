from .arm import Arm
from .WSG50Gripper import WSG50Gripper
from ..utils import math_util


class Kuka(Arm):

    def __init__(self, tool_id, engine,
                 path=None,
                 pos=(0., 0., 0.67),
                 orn=(0., 0., 0., 1.),
                 null_space=True,
                 gripper=None):
        path = path or 'kuka_iiwa/model_vr_limits.urdf'
        super(Kuka, self).__init__(tool_id, engine, path, pos, orn, null_space, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.025])
        self._rest_pose = (0., 0., 0., 1.5714, 0., -2.0944, 0.)
        self.reset()
