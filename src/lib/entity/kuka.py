import openravepy

from .arm import Arm
from .WSG50Gripper import WSG50Gripper

from ..utils import math_util


class Kuka(Arm):

    def __init__(self, tool_id, engine,
                 path=None,
                 ik_path=None,
                 pos=(0., 0., 0.67),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or 'kuka_iiwa/model_vr_limits.urdf'
        ik_path = ik_path or 'kuka_iiwa/kuka_arm.robot.xml'
        super(Kuka, self).__init__(
            tool_id, engine, path, ik_path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.045])
        self._rest_pose = (0., 0., 0., 1.570793, 0., -1.04719755, 0.)
        self.reset()

    @Arm.tool_orn.setter
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

        # Check if needs clip
        y, x = math_util.clip_vec(
            math_util.vec((orn[1], orn[0])),
            joint_spec['lower'][-2:],
            joint_spec['upper'][-2:])

        # Set the joints
        self.joint_states = (
            # Use last two DOFs
            [5, 6],
            [y, x], 'position',
            dict(positionGains=(.05,) * 2,
                 velocityGains=(1.,) * 2))

    def _build_ik(self, path, ik_path):

        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
        env = openravepy.Environment()

        env.Load(ik_path) # load a scene

        robot = env.GetRobots()[0] # get the first robot

        robot.SetActiveManipulator('arm') # set the manipulator

        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot, iktype=openravepy.IkParameterization.Type.Transform6D
        )

        if not ikmodel.load():
            ikmodel.autogenerate()
        return ikmodel

    def _move_to(self, pos, orn, ns=False):
        """
        Given pose, call IK to move
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :param ns: boolean indicating whether solve for
        null space solutions. Default is False.
        :return: None
        """
        specs = self.joint_specs

        # Clip the damping factors to make sure non-zero
        damps = math_util.clip_vec(specs['damping'], .1, 1.)
        if ns:
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
        else:
            ik_solution = self._engine.solve_ik(
                self._uid, self._end_idx,
                pos, orn=orn,
                damping=damps)

        self.joint_states = (
            self._joints, ik_solution, 'position',
            dict(forces=specs['max_force'],
                 positionGains=(.03,) * self._dof,
                 velocityGains=(1.,) * self._dof))
