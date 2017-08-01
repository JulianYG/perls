from .arm import Arm
from ..state.physicsEngine import OpenRaveEngine

from ..utils import math_util, io_util

import openravepy
from openravepy.misc import InitOpenRAVELogging 
InitOpenRAVELogging()


class Sawyer(Arm):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=(0., 0., 0.9),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or '../../data/sawyer_robot/sawyer_description/urdf/'
        super(Sawyer, self).__init__(
            tool_id, engine, path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.153])

        # Active joints indices: 5, 10, 11, 12, 13, 15, 18
        # Note the urdf has been modified and removed right_hand link, joint
        self._rest_pose = math_util.vec((0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           -1.18, 0.00, 2.18, 0.00,
                           0, 0.57, 0, 0, 3.3161))
        self._dof = 7
        self._active_joints = [5, 10, 11, 12, 13, 15, 18]

        self.reset()

    @property
    def pose(self):
        """
        Get the pose of sawyer arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        return self.kinematics['abs_frame_pos'][4], \
            self.kinematics['abs_frame_orn'][4]

    @Arm.tool_orn.setter
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
        if len(orn) == 4:
            orn = math_util.quat2euler(orn)
        
        joint_spec = self.joint_specs

        # Check if needs clip
        y, x = math_util.clip_vec(
            math_util.vec((orn[1], orn[0])),
            math_util.vec(joint_spec['lower'])[[15, 18]],
            math_util.vec(joint_spec['upper'])[[15, 18]])

        # Set the joints
        self.joint_states = (
            # Use last two DOFs
            [15, 18],
            [y, x], 'position',
            dict(positionGains=(.05,) * 2,
                 velocityGains=(1.,) * 2))

    def reset(self):
        """
        Reset tool to initial positions
        :return: None
        """
        super(Sawyer, self).reset()
        self._openrave_robot.SetDOFValues(self._rest_pose[[self._active_joints]], [1,2,3,4,5,6,7]) # Set the joint values

    def _build_ik(self, path_root):

        bullet_model_path = io_util.pjoin(path_root, 'sawyer.urdf')
        ikfast_model_path = io_util.pjoin(path_root, 'sawyer_arm.urdf')
        ikfast_base_path = io_util.pjoin(path_root, 'sawyer_base.srdf')

        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
        env = openravepy.Environment()
        plugin = openravepy.RaveCreateModule(env, "urdf")

        with env:
            name = plugin.SendCommand('load {} {}'.format(ikfast_model_path, ikfast_base_path))
            robot = env.GetRobot(name)
        
        robot.SetActiveManipulator('arm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                robot, iktype=openravepy.IkParameterization.Type.Transform6D
        )
        # Generate IK solver set if not already loaded
        if not ikmodel.load():
            ikmodel.autogenerate()
        return bullet_model_path, ikmodel, robot

    def _move_to(self, pos, orn, cc=True):

        # Convert to pose in robot base frame
        orn = self.kinematics['orn'][-1] if orn is None else orn

        import pybullet as p
        p.addUserDebugLine(pos, self.tool_pos, [1,0,0], 5, 3)
        pos, orn = math_util.get_relative_pose((pos, orn), self.pose)

        
        # openrave quaternion uses wxyz convention
        ik_solution = OpenRaveEngine.solve_ik(
            self._ik_model, pos, orn, 
            math_util.vec(self.joint_states['pos'])[self._active_joints])

        specs = self.joint_specs

        self.joint_states = (
            self._active_joints, ik_solution, 'position',
            dict(forces=math_util.vec(specs['max_force'])[self._active_joints],
                 positionGains=(.03,) * self._dof,
                 velocityGains=(1.,) * self._dof))
        # Set the joint values
        self._openrave_robot.SetDOFValues(ik_solution, [1,2,3,4,5,6,7]) 

        while math_util.pos_diff(math_util.vec(self.joint_states['pos'])[self._active_joints], ik_solution, 0) > .01:
            self._engine.hold()
