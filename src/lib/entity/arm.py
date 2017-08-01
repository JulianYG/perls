import abc

from ..state.physicsEngine import OpenRaveEngine

from .body import Tool
from ..utils import math_util


class Arm(Tool):

    def __init__(self, tid, engine, path_root, 
                 pos, orn, collision_checking,
                 gripper):
        """
        Initialization 
        :param path: arm model asset file path
        :param pos: initial position vec3 float cartesian
        :param orn: initial orientation vec4 float quat
        """
        path, self._ik_model, self._openrave_robot, self._model_dof = \
            self._build_ik(path_root)
        super(Arm, self).__init__(tid, engine, path, pos, orn, fixed=True)

        # Reset pose is defined in subclasses
        self._gripper = gripper
        self._rest_pose = (0., ) * self._dof
        self._end_idx = self._dof - 1

        self.collision_checking = collision_checking

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: integer tool id
        """
        return 'm{}'.format(self._tool_id)

    @abc.abstractproperty
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        raise NotImplemented

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
        return self.kinematics['orn'][-1]

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
        self._move_to(target_pos, None, False, True)

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the tool to given orientation.
        Note setting orientation does not keep previous
        position. Due to the limitation of common
        robot arms, it preserves [roll, pitch] but
        abandons [yaw].
        :param orn: vec4 float in quaternion form,
        or vec3 float in euler radian
        :return: None
        """
        if len(orn) == 4:
            orn = math_util.quat2euler(orn)
        joint_spec = self.joint_specs
        eef_joints = self.active_joints[-2:]

        # Set the joints if within range
        if joint_spec['lower'][eef_joints[0]] <= orn[0] \
                <= joint_spec['upper'][eef_joints[0]]:
            self.joint_states = (
                # Use last two DOFs
                [eef_joints[0]],
                [orn[0]], 'position',
                dict(positionGains=(.05,),
                     velocityGains=(1.,)))

        if joint_spec['lower'][eef_joints[1]] <= orn[1] \
                <= joint_spec['upper'][eef_joints[1]]:
            self.joint_states = (
                # Use last two DOFs
                [eef_joints[1]],
                [orn[1]], 'position',
                dict(positionGains=(.05,),
                     velocityGains=(1.,)))

    @abc.abstractmethod
    def _build_ik(self, path_root):
        """
        Build the ik model for the arm.
        :param path_root: The absolute path directory that stores 
        pybullet asset file, and IKFast model file, base files.
        :return: pybullet asset file path, 
        and the built IKFast model.
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
        end_effector_info = self.kinematics

        end_effector_pos, end_effector_orn = \
            end_effector_info['pos'][-1], \
            end_effector_info['orn'][-1]

        # Repeat same procedure for gripper base link
        # and arm end effector link
        transform = math_util.pose2mat(
            math_util.get_relative_pose(
                (end_effector_pos, end_effector_orn),
                (self.tool_pos, self.tool_orn))
            )
        frame = math_util.pose2mat((pos, orn)).dot(transform)
        return math_util.mat2pose(frame)

    def _move_to(self, pos, orn, precise, fast):
        """
        Given pose, call IK to move
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param precise: boolean indicating whether
        using precise IK for motion planning. If
        true, reaches millimeter level accuracy. However
        this may not be necessary for real time control,
        in order to achieve better performance in time
        and smoothness.
        ***This is a trade-off between smoothness and
         accuracy***
        :param fast: refer to <pinpoint::fast>
        :return: None
        """
        # Convert to pose in robot base frame
        orn = self.kinematics['orn'][-1] if orn is None else orn
        #
        # import pybullet as p
        # p.addUserDebugLine(pos, self.tool_pos, [1,0,0], 5, 3)

        specs = self.joint_specs
        if precise:
            if fast:
                # Set the joint values in openrave model
                self._openrave_robot.SetDOFValues(
                    math_util.vec(self.joint_states['pos'])[self.active_joints],
                    self._model_dof)

            pos, orn = math_util.get_relative_pose((pos, orn), self.pose)
            ik_solution = OpenRaveEngine.accurate_ik(
                self._ik_model, pos, orn,
                math_util.vec(self.joint_states['pos'])[self.active_joints],
                closest=not fast
            )

        else:
            damps = math_util.clip_vec(specs['damping'], .1, 1.)
            lower_limits = math_util.vec(specs['lower'])
            upper_limits = math_util.vec(specs['upper'])
            ranges = upper_limits - lower_limits

            # Solve using null space
            ik_solution = self._engine.solve_ik_null_space(
                self._uid, self._end_idx,
                pos, orn=orn,
                lower=lower_limits,
                upper=upper_limits,
                ranges=ranges,
                rest=self._rest_pose,
                damping=damps)

        self.joint_states = (
            self.active_joints, ik_solution, 'position',
            dict(forces=math_util.vec(specs['max_force'])[self.active_joints],
                 positionGains=(.03,) * self._dof,
                 velocityGains=(1.,) * self._dof))

        # TODO :
        # if self.collision_checking:
        # need to wait until reached desired states, just as in real case
        while math_util.pos_diff(math_util.vec(self.joint_states['pos'])[self.active_joints],
                                 ik_solution, 0) > .01:
            # self._engine.hold()
            continue

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

    def pinpoint(self, pos, orn, ftype='abs',
                 fast=False):
        """
        Accurately reach to the given pose.
        Note this operation sets position and orientation
        and the same time, keeping neither previous
        position nor previous orientation
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param ftype: refer to <reach~ftype>
        :param fast: boolean indicating whether use fast
        solution. Fast requires less computation
        time, but may result in big arm movements. If set
        to false, the solutions are weighted to find the
        closest neighbor of current joint states.
        ***This is a trade-off between smoothness and
        speed***
        :return: delta between target and actual pose
        """
        target_pos, target_orn = super(Arm, self).pinpoint(pos, orn, ftype)
        self._move_to(target_pos, target_orn, True, fast)

        pos_delta = self.tool_pos - pos
        orn_delta = math_util.quat_diff(self.tool_orn, orn)
        return pos_delta, orn_delta

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        self._gripper.grasp(slide)
