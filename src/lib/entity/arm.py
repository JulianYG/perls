import abc

from .body import Tool
from ..utils import math_util
from ..utils.io_util import loginfo, FONT


class Arm(Tool):

    def __init__(self, tid, engine, path,
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
        self._gripper = gripper
        self._rest_pose = (0., ) * self._dof
        self._end_idx = self.active_joints[-1]

        self.collision_checking = collision_checking

    @property
    def type(self):
        """
        Get the type of the body.
        :return: string
        """
        return 'arm'

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: integer tool id
        """
        return 'm{}'.format(self._tool_id)

    @abc.abstractproperty
    def close_grip(self):
        """
        Check if the gripper of arm is closed
        :return: boolean
        """
        if self._gripper:
            return self._gripper.close_grip
        else:
            return True

    @abc.abstractproperty
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        return NotImplemented

    @property
    def pose(self):
        """
        Get the pose of arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        kinematics = self.kinematics
        return kinematics['abs_frame_pos'][0], \
            kinematics['abs_frame_orn'][0]

    @property
    def eef_pose(self):
        """
        Get the end effector pose of the robot.
        :return: (pos, orn) tuple of the end effector
        """
        kinematics = self.kinematics
        return kinematics['abs_frame_pos'][self._end_idx], \
            kinematics['abs_frame_orn'][self._end_idx]

    @property
    def tool_pos(self):
        """
        Get the position of the tool. This is semantic
        :return: vec3 float in Cartesian
        """
        if self._gripper:
            return self._gripper.tool_pos
        else:
            return self.kinematics['pos'][self._end_idx]

    @property
    def tool_orn(self):
        """
        Get the orientation of the gripper attached at
        robot end effector link. There should exist
        some offset based on robot's rest pose.
        :return: vec4 float quaternion in Cartesian
        """
        return self.kinematics['orn'][self._end_idx]

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos -
                             self.kinematics['pos'][self._end_idx])

    @Tool.v.getter
    def v(self):
        """
        For robot arms, get the end effector linear velocityGainsy
        :return: vec3 float cartesian
        """
        if self._gripper:
            return self._gripper.v
        else:
            return self.kinematics['abs_v'][self._end_idx]

    @Tool.omega.getter
    def omega(self):
        """
        Get the end effector angular velocity
        :return: vec3 radian
        """
        if self._gripper:
            return self._gripper.omega
        else:
            return self.kinematics['abs_omega'][self._end_idx]

    @tool_pos.setter
    def tool_pos(self, pos_iter):
        """
        Set the tool to given pose.
        :param pos_iter: 
        tuple of (pos, use_iter) 
        (vec3 float in cartesian space, boolean use iteration)
        referring to the position between the gripper fingers.
        Note it only controls the position of the gripper,
        and does not keep the orientation.
        :return: None
        """
        pos, use_iter = pos_iter
        if self._gripper:
            target_pos, _ = self.position_transform(pos, self.tool_orn)
        else:
            target_pos = pos

        if use_iter is not None:
            self._move_to(target_pos, None,
                          precise=False,
                          fast=True,
                          iterative=True,
                          max_iter=use_iter,
                          threshold=1e-2,
                          ctype='position')
        else:
            self._move_to(target_pos, None,
                          precise=False,
                          fast=True,
                          iterative=False,
                          max_iter=0,
                          threshold=1e-2,
                          ctype='position')

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the tool to given orientation.
        Note setting orientation does not keep previous
        position. Due to the limitation of common
        robot arms, it preserves [roll, pitch] but
        abandons [yaw].
        :param orn: vec4 float in quaternion form;
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
            jpos = [None] * self._dof
            jpos[eef_joints[0]] = orn[0]
            self.joint_positions = (
                jpos, dict(positionGains=(.05,),
                           velocityGains=(1.,))
            )

        if joint_spec['lower'][eef_joints[1]] <= orn[1] \
                <= joint_spec['upper'][eef_joints[1]]:
            jpos = [None] * self._dof
            jpos[eef_joints[1]] = orn[1]
            self.joint_positions = (
                jpos, dict(positionGains=(.05,),
                           velocityGains=(1.,))
            )

    def set_eef_pose(self, pos, orn, iters=500):
        """
        Directly move the end effector to desired pose.
        Note that it matches the reference frame pose.
        :param pos: position cartesian vec float 3
        :param orn: orientation quaternion vec float 4
        :param iters: number of iterations to reach for 
        the end effector pose 
        """
        self._move_to(pos, orn,
                      precise=False,
                      fast=True,
                      iterative=iters > 0,
                      max_iter=iters,
                      threshold=1e-2,
                      ctype='position')

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
        end_effector_pos, end_effector_orn = self.eef_pose

        # Repeat same procedure for gripper base link
        # and arm end effector link
        transform = math_util.pose2mat(
            math_util.get_relative_pose(
                (end_effector_pos, end_effector_orn),
                (self.tool_pos, self.tool_orn))
            )
        frame = math_util.pose2mat((pos, orn)).dot(transform)
        return math_util.mat2pose(frame)

    def _move_to(self, pos, orn, precise,
                 fast,  # Equivalent to !Null_Space
                 iterative, max_iter, threshold, ctype):
        """
        Given pose, call IK to move
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param precise: boolean indicating whether
        using precise IK for motion planning. If
        both precise and iterative are true,
        max_iter param will be used as timeout, and
        iterate until error is below threshold. However
        this may not be necessary for real time control,
        in order to achieve better performance in time
        and smoothness.
        ***This is a trade-off between smoothness and
         accuracy***
        :param fast: refer to <pinpoint::fast>
        :param iterative: Boolean indicating if this
        arm uses iterative solver
        :param max_iter: refer to <pinpoint::max_iter>
        :param ctype: the control type. Specify among
        <'position', 'velocity', 'torque'> to perform certain
        controlling.
        :return: None
        """
        # Convert to pose in robot base frame
        orn = self.kinematics['abs_frame_orn'][self._end_idx]\
            if orn is None else orn
        specs = self.joint_specs
        damps = math_util.clip_vec(specs['damping'], .1, 1.)
        lower_limits = math_util.vec(specs['lower'])
        upper_limits = math_util.vec(specs['upper'])
        ranges = upper_limits - lower_limits

        if ctype == 'position':

            def _position_control_helper():

                """
                Given pose, solve for accurate joint positions and
                move there with BulletIK. Helper for iterative
                """
                if fast:
                    ik_solution = self._engine.solve_ik(
                        self._uid, self._end_idx, pos, damps, orn=orn)
                else:
                    # Solve using null space
                    ik_solution = self._engine.solve_ik_null_space(
                        self._uid, self._end_idx,
                        pos, orn=orn,
                        lower=lower_limits,
                        upper=upper_limits,
                        ranges=ranges,
                        rest=self._rest_pose,
                        damping=damps)

                # TODO: collision checking?

                self.joint_positions = (
                    ik_solution,
                    dict(positionGains=(.05,) * self._dof,
                         velocityGains=(1.,) * self._dof)
                )

            # Iteratively move to desired position
            if iterative:
                if precise:
                    steps = 0
                    # Iterate until error below threshold
                    while math_util.rms(self.tool_pos - pos) > threshold:
                        _position_control_helper()
                        # Timeout
                        if steps > max_iter:
                            break
                        steps += 1
                    max_iter = steps
                else:
                    for _ in range(max_iter):
                        _position_control_helper()
                        self._engine.hold(1)
            else:
                _position_control_helper()
                max_iter = 1

        # elif ctype == 'torque':
        # TODO: torque Operational Space Control

        return max_iter
        
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

        self._engine.hold()

        # Lastly reset arm
        self.joint_positions = (
             self._rest_pose,
             dict(reset=True))

    def pinpoint(self, pos, orn,
                 ftype='abs',
                 iterative=True,
                 fast=False,
                 max_iter=200,
                 threshold=1e-2,
                 ctype='position'):
        """
        Accurately reach to the given pose.
        Note this operation sets position and orientation
        and the same time, keeping neither previous
        position nor previous orientation
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param ftype: refer to <reach~ftype>
        :param fast: boolean indicating whether use null space
        solution. If set true, Fast requires less computation
        time, but may result in big arm movements. If set
        to false, the solutions are weighted to find the
        closest neighbor of current joint states, and are
        closer to the rest pose of the arm.
        ***This is a trade-off between smoothness and
        speed***
        :param iterative: refer to <_move_to::iterative>
        :param max_iter: maximum iterations in either real 
        time or non-real time simulation for the arm to
        reach the desired position.
        :param threshold: error threshold
        :param ctype: control type
        :return: number of iterations used for pinpoint
        """
        target_pos, target_orn = super(Arm, self).pinpoint(pos, orn, ftype)
        return self._move_to(target_pos, target_orn, True, fast, iterative,
                             max_iter, threshold, ctype)

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        if self._gripper:
            self._gripper.grasp(slide)
