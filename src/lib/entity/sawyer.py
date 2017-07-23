from .arm import Arm
from .rethinkGripper import RethinkGripper
from ..utils import math_util


class Sawyer(Arm):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=(0., 0., 0.9),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or 'sawyer_robot/sawyer_description/urdf/sawyer.urdf'
        super(Sawyer, self).__init__(
            tool_id, engine, path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.155])

        # Active joints indices: 5, 10, 11, 12, 13, 15, 18
        # Note the urdf has been modified and removed right_hand link, joint
        self._rest_pose = [0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           -1.18, 0.00, 2.18, 0.00,
                           0, 0.57, 0, 0, 3.3161]
        self.reset()

    @property
    def pose(self):
        """
        Get the pose of sawyer arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        return self.kinematics['abs_frame_pos'][3], \
            self.kinematics['abs_frame_orn'][3]