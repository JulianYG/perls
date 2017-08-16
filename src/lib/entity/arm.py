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
        return kinematics['pos'][self._end_idx], \
            kinematics['orn'][self._end_idx]

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
        return self.kinematics['orn'][self._end_idx]

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos - self.kinematics['pos'][self._end_idx])

    @Tool.v.getter
    def v(self):
        """
        For robot arms, get the end effector linear velocityGainsy
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
        kinematics = self.kinematics

        end_effector_pos, end_effector_orn = \
            kinematics['abs_frame_pos'][self._end_idx], \
            kinematics['abs_frame_orn'][self._end_idx]

        # Repeat same procedure for gripper base link
        # and arm end effector link
        transform = math_util.pose2mat(
            math_util.get_relative_pose(
                (end_effector_pos, end_effector_orn),
                (self.tool_pos, self.tool_orn))
            )
        frame = math_util.pose2mat((pos, orn)).dot(transform)
        return math_util.mat2pose(frame)

    def _move_to(self, pos, orn, precise, fast,
                 max_iter=200, ctype='position'):
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
        :param max_iter: refer to <pinpoint::max_iter>
        :param ctype: the control type. Specify among
        <'position', 'velocity', 'torque'> to perform certain
        controlling.
        :return: None
        """
        # Convert to pose in robot base frame
        orn = self.kinematics['abs_frame_orn'][self._end_idx] if orn is None else orn
        specs = self.joint_specs

        # if precise:
        #     if fast or self.collision_checking:
        #         # Set the joint values in openrave model
        #         self._openrave_robot.SetDOFValues(
        #             math_util.vec(self.joint_states['pos'])[self.active_joints],
        #             self._model_dof)
        #
        #     pos, orn = math_util.get_relative_pose((pos, orn), self.pose)
        #
        #     ik_solution = OpenRaveEngine.accurate_ik(
        #         self._ik_model, pos, orn,
        #         math_util.vec(self.joint_states['pos'])[self.active_joints],
        #         closest=not fast)
        # else:
        damps = math_util.clip_vec(specs['damping'], .1, 1.)
        lower_limits = math_util.vec(specs['lower'])
        upper_limits = math_util.vec(specs['upper'])
        ranges = upper_limits - lower_limits

        if ctype == 'position':
            # Solve using null space
            ik_solution = self._engine.solve_ik_null_space(
                self._uid, self._end_idx,
                pos, orn=orn,
                lower=lower_limits,
                upper=upper_limits,
                ranges=ranges,
                rest=self._rest_pose,
                damping=damps)

            # TODO :
            # if self.collision_checking:

            #     request = {
            #       "basic_info" : {
            #         "n_steps" : 10,
            #         "manip" : "rightarm", # see below for valid values
            #         "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
            #       },
            #       "costs" : [
            #       {
            #         "type" : "joint_vel", # joint-space velocity cost
            #         "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
            #         # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
            #       },
            #       {
            #         "type" : "collision",
            #         "params" : {
            #           "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
            #           "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
            #         },
            #       }
            #       ],
            #       "constraints" : [
            #       {
            #         "type" : "joint", # joint-space target
            #         "params" : {"vals" : ik_solution} # length of vals = # dofs of manip
            #       }
            #       ],
            #       "init_info" : {
            #           "type" : "straight_line", # straight line in joint space.
            #           "endpoint" : ik_solution
            #       }
            #     }

            self.joint_positions = (
                ik_solution,
                dict(positionGains=(.05,) * self._dof,
                     velocityGains=(1.,) * self._dof)
            )
        # TODO

        # elif ctype == 'torque':

        # s = json.dumps(request) # convert dictionary into json-formatted string
        # prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
        # t_start = time.time()
        # result = trajoptpy.OptimizeProblem(prob) # do optimization
        # t_elapsed = time.time() - t_start
        # print result
        # print "optimization took %.3f seconds"%t_elapsed

        # from trajoptpy.check_traj import traj_is_safe
        # prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        # assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

        # need to wait until reached desired states, just as in real case
        # for _ in range(max_iter):
        #     self._engine.step(0)
        
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

    def pinpoint(self, pos, orn, ftype='abs',
                 fast=False, max_iter=200):
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
        :param max_iter: maximum iterations in either real 
        time or non-real time simulation for the arm to
        reach the desired position.
        :return: rms delta between target and actual pose
        """
        if ftype == 'abs':
            orig_pos, orig_orn = pos, orn

        elif ftype == 'rel':
            orig_pos, orig_orn = math_util.get_absolute_pose(
                (pos, orn), self.pose)
        else:
            loginfo('Cannot recognize frame type, assuming absolute',
                    FONT.ignore)
            orig_pos, orig_orn = pos, orn
        
        target_pos, target_orn = super(Arm, self).pinpoint(pos, orn, ftype)
        self._move_to(target_pos, target_orn, True, fast, max_iter)

        # TODO: orn diff flip
        return math_util.pose_diff(self.tool_pose, (orig_pos, orig_orn))

    def grasp(self, slide=-1):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        self._gripper.grasp(slide)
