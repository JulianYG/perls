
from .body import Tool
from ..utils import math_util


class Arm(Tool):

    def __init__(self, tid, engine, path,
                 pos, orn, null_space,
                 gripper):
        """
        Initialization 
        :param path: arm model asset file path
        :param pos: initial position vec3 float cartesian
        :param orn: initial orientation vec4 float quat
        """
        super(Arm, self).__init__(tid, engine, path, pos, orn, fixed=True)

        # Reset pose is defined in subclasses
        self._gripper = gripper
        self._rest_pose = (0., ) * self._dof
        self._end_idx = self._dof - 1
        self._null_space = null_space

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: integer tool id
        """
        return 'm{}'.format(self._tool_id)

    @property
    def null_space(self):
        """
        Property saying if the robot is using null space to
        solve IK
        :return: Boolean
        """
        return self._null_space

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
        Get the orientation of the gripper attached at
        robot end effector link. There should exist
        some offset based on robot's rest pose.
        :return: vec4 float quaternion in Cartesian
        """
        return self._gripper.tool_orn

    @null_space.setter
    def null_space(self, use_ns):
        """
        Set robot to use null space IK solver or not
        :param use_ns: True/false
        :return: None
        """
        self._null_space = use_ns

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the tool to given pose.
        :param pos: vec3 float in cartesian space,
        referring to the position between the gripper fingers
        :return: None
        """
        target_pos = self.position_transform(pos, self.tool_orn)
        self._move_to(target_pos, self.tool_orn)

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the tool to given orientation.
        Note setting orientation does not keep previous
        position. Due to the limitation of common
        robot arms, it preserves [roll, pitch] but
        abandons [yaw].
        :param orn: vec4 float in quaternion form,
        or vec3 float in euler form
        :return: None
        """
        orn = math_util.quat2euler(orn)
        # Check if needs clip
        y, x, _ = math_util.joint_clip(
            math_util.vec((orn[1], orn[0], orn[2])), self.joint_specs)

        # Set the joints
        self.joint_states = (
            # Use last two DOFs
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
        rotation = math_util.quat2mat(orn)
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
    #  High level functionality

    def reset(self):
        """
        Reset tool to initial positions
        :return: None
        """
        # Reset arm
        self.joint_states = \
            (self._joints,
             self._rest_pose,
             'position',
             dict(positionGains=(.05,) * self._dof,
                  velocityGains=(1.,) * self._dof))
        if self._gripper:
            # Attach gripper
            self.attach_children = \
                (self._end_idx,
                 self._gripper.uid,
                 0, 'fixed',
                 [0., 0., 0.], self._tip_offset,
                 [0., 0., 0.],
                 # TODO: Check if can use [0., 0., 0.707, 0.707] for child orn
                 [0., 0., 0., 1.], [0., 0., 0., 1.])
            # Reset gripper
            self._gripper.reset()
        self._engine.hold()

    def reach(self, pos=None, orn=None):
        """
        Reach to given pose.
        Note this operation sets position first, then
        adjust to orientation by rotation end effector,
        so the position is not accurate in a sense.
        For accurate control, call <pinpoint> instead.
        :param pos: vec3 float cartesian
        :param orn: vec3 float quaternion
        :return: delta between target and actual pose
        """
        orn_delta = math_util.zero_vec(3)
        pos_delta = math_util.zero_vec(3)

        if pos is not None:
            self._move_to(pos, orn)
            pos_delta = self.tool_pos - pos

        if orn is not None:
            self.tool_orn = orn
            orn_delta = math_util.quat_diff(self.tool_orn, orn)

        return pos_delta, orn_delta

    def pinpoint(self, pos, orn=None):
        """
        Accurately reach to the given pose.
        Note this operation sets position and orientation
        and the same time, keeping neither previous
        position nor previous orientation
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :return: None
        """
        if orn is None:
            orn = self.tool_orn
        target_pos = self.position_transform(pos, orn)
        self._move_to(target_pos, orn)

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        self._gripper.grasp(slide)
