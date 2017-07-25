from .arm import Arm
from ..state.physicsEngine import OpenRaveEngine
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

    def _move_to(self, pos, orn, cc=True):
        # Convert to pose in robot base frame
        pos, _ = math_util.get_transformed_pose((pos, orn), self.pose)

        # TODO: verify orientation mismatch
        hmat = math_util.pose2mat((pos, orn))
        indices = [5, 10, 11, 12, 13, 15, 18]
        ik_solution = OpenRaveEngine.solve_ik(
            self._arm_model, pos, orn, self.joint_states[indices])

        specs = self.joint_specs

        self.joint_states = (
            self._joints, ik_solution, 'position',
            dict(forces=specs['max_force'],
                 positionGains=(.03,) * self._dof,
                 velocityGains=(1.,) * self._dof))