from .arm import Arm
from ..utils import math_util


class Sawyer(Arm):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=(0., 0., 0.9),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or 'sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf'
        super(Sawyer, self).__init__(
            tool_id, engine, path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.153])

        # Note the urdf has been modified and removed right_hand link & joint
        self._rest_pose = math_util.vec((0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161))

        self._dof = 7
        self.reset()

    @property
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        return range(7)

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos - self.kinematics['pos'][self._end_idx]) * 2
