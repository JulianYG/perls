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

    @property
    def pose(self):
        """
        Get the pose of sawyer arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        # TODO : Change to correct link index # 3, pedestal fixed
        return self.kinematics['abs_frame_pos'][0], \
            self.kinematics['abs_frame_orn'][0]