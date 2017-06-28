
from .body import Tool
from ..utils import math_util


class Arm(Tool):

    def __init__(self, engine, path,
                 pos, orn, null_space,
                 gripper):
        """
        Initialization 
        :param path: arm model asset file path
        :param pos: initial position vec3 float cartesian
        :param orn: initial orientation vec4 float quat
        """
        Tool.__init__(self, engine, path, pos, orn, fixed=True)

        # Reset pose is defined in subclasses
        self._gripper = gripper
        self._rest_pose = (0., ) * self._dof
        self._end_idx = self._dof - 1
        self._null_space = null_space

    @property
    def tool_pos(self):
        """
        Get the position of the tool. This is semantic
        :return: vec3 float in Cartesian
        """
        return self._gripper.tool_pos

    @property
    def tool_orn(self):
        """
        Get the orientation of the tool. This is semantic;
        directly use the orientation of end effector link 
        since it's fixed with the gripper
        :return: vec4 float quaternion in Cartesian
        """
        return self.kinematics['orn'][self._end_idx]

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the tool to given pose.
        Note setting position does not keep previous 
        orientation by forcing it to (0, 1, 0, 0)
        To preserve orientation, use 
        higher level methods such as <reach>
        :param pos: vec3 float in cartesian space,
        referring to the position between the gripper fingers
        :return: None
        """
        target_pos = self.position_transform(pos, self.tool_orn)
        self._move_to(target_pos, None)

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the tool to given orientation.
        Note setting orientation does not keep previous
        position. Due to the limitation of common 
        robot arms, it preserves [roll, pitch] but 
        abandons [yaw].
        :param orn: vec4 float in quaternion form
        :return: None
        """
        x, y, _ = math_util.quat2euler(orn)
        self.joint_states = (
            # Use last two dofs
            self._joints[-2:],
            [y, x], 'position',
            dict(positionGains=(.05,) * 2,
                 velocityGains=(1.,) * 2))

    ###
    # Helper functions
    def position_transform(self, pos, orn):
        """
        Helper function to convert position between 
        fingers of gripper to position on robot 
        end effector link
        :param pos: vec3 float cartesian world frame
        :param orn: vec4 float quaternion world frame
        :return: vec3 float cartesian world frame
        """
        # Get Center of Mass (CoM) of averaging
        # left/right gripper fingers
        # First get position of gripper base
        gripper_base_pos = \
            self._gripper.position_transform(pos, orn)

        # Repeat same procedure for gripper base link
        # and arm end effector link
        gripper_arm_tran = gripper_base_pos - pos

        # Math:
        # relative: transformed_pos = rotation x (pos - translation)
        # absolute: transformed_pos = pos - rotation x abs_frame_orn
        rotation = math_util.get_rotation_from_quaternion(orn)
        target_pos = gripper_base_pos - rotation.dot(gripper_arm_tran)
        return target_pos

    def _move_to(self, pos, orn):
        """
        Given pose, call IK to move
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :return: None
        """
        specs = self.joint_specs
        damps = specs['damping']

        if self._null_space:

            lower_limits = math_util.vec(specs['lower'])
            upper_limits = math_util.vec(specs['upper'])
            ranges = upper_limits - lower_limits
            ik_solution = self._engine.solve_ik_null_space(
                self._uid, self._end_idx,
                pos, orn=orn,
                lower=lower_limits,
                upper=upper_limits,
                ranges=ranges,
                rest=self._rest_pose,
                damping=damps)
        else:
            ik_solution = self._engine.solve_ik(
                self._uid, self._end_idx,
                pos, orn=orn,
                damping=damps)

        self.joint_states = (
            self._joints, ik_solution, 'position',
            dict(forces=specs['max_force'],
                 positionGains=(.05,) * self._dof,
                 velocityGains=(1.,) * self._dof))

    ###
    #  High level functionalities
    def reset(self):
        """
        Reset tool to initial positions
        :return: None
        """
        # Reset gripper
        if self._gripper:
            self._gripper.reset()
            # Attach gripper
            self.attach_children = \
                (self._dof - 1,
                 self._gripper.uid,
                 0, 'fixed',
                 [0., 0., 0.], self._tip_offset,
                 [0., 0., 0.],
                 [0., 0., 0., 1.], [0., 0., .707, .707])

        # Reset arm
        self.joint_states = \
            (self._joints,
             self._rest_pose,
             'position',
             dict(positionGains=(.05,) * self._dof,
                  velocityGains=(1.,) * self._dof))

        self._engine.update()

    def reach(self, pos, orn=None):
        """
        Reach to given pose. 
        Note this operation sets position and orientation
        and the same time, keeping neither previous
        position nor previous orientation
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :return: delta between target and actual pose
        """
        target_pos = self.position_transform(pos, orn or self.tool_orn)
        self._move_to(target_pos, orn or self.tool_orn)
        orn_delta = self.tool_orn - (orn or self.tool_orn)
        pos_delta = self.tool_pos - pos
        return pos_delta, orn_delta

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        self._gripper.grasp(slide)

