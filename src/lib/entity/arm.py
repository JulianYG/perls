import abc

from .body import Tool
from ..utils import math_util


class Arm(Tool):

    def __init__(self, tid, engine, path, ik_path,
                 pos, orn, collision_checking,
                 gripper):
        """
        Initialization 
        :param path: arm model asset file path
        :param pos: initial position vec3 float cartesian
        :param orn: initial orientation vec4 float quat
        """
        super(Arm, self).__init__(tid, engine, path, pos, orn, fixed=True)

        # Reset pose is defined in subclasses
        self._ik_model = self._build_ik(path, ik_path)
        self._gripper = gripper
        self._rest_pose = (0., ) * self._dof
        self._end_idx = self._dof - 1
        self._collision_checking = collision_checking

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: integer tool id
        """
        return 'm{}'.format(self._tool_id)

    @property
    def collision_checking(self):
        """
        Property saying if the robot is using null space to
        solve IK
        :return: Boolean
        """
        return self._collision_checking

    @property
    def pose(self):
        """
        Get the pose of arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        return self.kinematics['abs_frame_pos'][0], \
            self.kinematics['abs_frame_orn'][0]

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

    @Tool.v.getter
    def v(self):
        """
        For robot arms, get the end effector linear velocity
        :return: vec3 float cartesian
        """
        return self._gripper.v

    @Tool.omega.getter
    def omega(self):
        """
        Get the end effector angular velocity
        :return: vec3 radian
        """
        return self._gripper.omega

    @collision_checking.setter
    def collision_checking(self, collision_checking):
        """
        Set robot to use null space IK solver or not
        :param collision_checking: True/false
        :return: None
        """
        self._collision_checking = collision_checking

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the tool to given pose.
        :param pos: vec3 float in cartesian space,
        referring to the position between the gripper fingers.
        Note it only controls the position of the gripper,
        and does not keep the orientation.
        :return: None
        """
        target_pos, _ = self.position_transform(pos, self.tool_orn)
        self._move_to(target_pos, None, self._collision_checking)

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
        raise NotImplemented

    @abc.abstractmethod
    def _build_ik(self, path, ik_path):
        """
        Build the ik model for the arm.
        :return: The built IK model in openrave.
        """
        raise NotImplemented

    def get_pose(self, uid=None, lid=None):
        """
        Get the current base pose of the tool. This is
        especially useful for end effector pose relative
        to
        :return: (pos, orn) tuple
        """
        if uid:
            frame_pos = self._engine.get_body_scene_position(uid)
            frame_orn = self._engine.get_body_scene_orientation(uid)
            if lid:
                frame_pos, frame_orn = \
                    self._engine.get_body_link_state(uid, lid)[:2]

            return self._engine.get_body_relative_pose(
                self._uid, frame_pos, frame_orn)
        else:
            return self.pose

    ###
    # Helper functions
    def position_transform(self, pos, orn):
        """
        Helper function to convert position between
        fingers of gripper to position on robot
        end effector link
        :param pos: vec3 float cartesian world frame
        :param orn: vec4 float quaternion world frame
        :return: transformed pose (pos, orn) in world frame
        """
        # Get Center of Mass (CoM) of averaging
        # left/right gripper fingers
        # First get position of gripper base

        # gripper_base_pos, gripper_base_orn = \
        #     self._gripper.position_transform(pos, orn)
        # print(pos, gripper_base_pos, 'here')

        end_effector_info = self.kinematics

        end_effector_pos, end_effector_orn = \
            end_effector_info['pos'][19], \
            end_effector_info['orn'][19]

        # Repeat same procedure for gripper base link
        # and arm end effector link
        transform = math_util.pose2mat(
            math_util.get_relative_pose(
                (end_effector_pos, end_effector_orn),
                (self.tool_pos, self.tool_orn))
            )
        frame = math_util.pose2mat((pos, orn)).dot(transform)
        return math_util.mat2pose(frame)



        # gripper_arm_tran = gripper_base_pos - pos

        # # Math:
        # # relative: transformed_pos = rotation x (pos - translation)
        # # absolute: transformed_pos = pos - rotation x abs_frame_orn
        # rotation = math_util.quat2mat(orn)
        # target_pos = gripper_base_pos - rotation.dot(gripper_arm_tran)
        # return target_pos

    @abc.abstractmethod
    def _move_to(self, pos, orn, ns=False):
        """
        Given pose, call IK to move
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param ns: boolean indicating whether solve for
        null space solutions. Default is False.
        :return: None
        """
        raise NotImplemented

    ###
    #  High level functionality
    def reset(self):
        """
        Reset tool to initial positions
        :return: None
        """
        if self._gripper:
            # First attach gripper
            self.attach_children = \
                (self._end_idx,
                 self._gripper.uid,
                 0, 'fixed',
                 [0., 0., 0.], self._tip_offset,
                 [0., 0., 0.],
                 [0., 0., 0., 1.], [0., 0., 0., 1.])
            # Next reset gripper
            self._gripper.reset()

        # Lastly reset arm
        self.joint_states = \
            (self._joints,
             self._rest_pose,
             'position',
             dict(reset=True))

        import pybullet as p
        p.addUserDebugLine(self.tool_pos,
        self.position_transform(self.tool_pos, self.tool_orn), [1, 0, 0], 5, 50)

    def pinpoint(self, pos, orn, ftype='abs'):
        """
        Accurately reach to the given pose.
        Note this operation sets position and orientation
        and the same time, keeping neither previous
        position nor previous orientation
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param ftype: refer to <reach~ftype>
        :return: delta between target and actual pose
        """
        target_pos, target_orn = super(Arm, self).pinpoint(pos, orn, ftype)
        self._move_to(target_pos, target_orn)

        pos_delta = self.tool_pos - fpos
        orn_delta = math_util.quat_diff(self.tool_orn, forn)
        return pos_delta, orn_delta

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        self._gripper.grasp(slide)
