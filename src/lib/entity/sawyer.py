from .arm import Arm
from .rethinkGripper import RethinkGripper
from ..utils import math_util


class Sawyer(Arm):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=(0., 0., 0.9),
                 orn=(0., 0., 0., 1.),
                 null_space=True,
                 gripper=None):
        path = path or 'sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf'
        super(Sawyer, self).__init__(tool_id, engine, path, pos, orn, null_space, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.155])
        self._rest_pose = (0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161)
        self.reset()
