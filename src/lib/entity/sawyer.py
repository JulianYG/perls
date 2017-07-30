from .arm import Arm
from ..state.physicsEngine import OpenRaveEngine
from ..utils import math_util
import openravepy

class Sawyer(Arm):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 ik_path=None,
                 pos=(0., 0., 0.9),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or 'sawyer_robot/sawyer_description/urdf/sawyer.urdf'
        ik_path = ik_path or 'sawyer_robot/sawyer_description/urdf/sawyer_base.srdf'
        super(Sawyer, self).__init__(
            tool_id, engine, path, ik_path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.155])

        # Active joints indices: 5, 10, 11, 12, 13, 15, 18
        # Note the urdf has been modified and removed right_hand link, joint
        self._rest_pose = [0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           -1.18, 0.00, 2.18, 0.00,
                           0, 0.57, 0, 0, 3.3161, 0]
        self._dof = 7
        self.reset()

    @property
    def pose(self):
        """
        Get the pose of sawyer arm base frame.
        :return: arm base frame (pos, orn) tuple
        """
        return self.kinematics['abs_frame_pos'][3], \
            self.kinematics['abs_frame_orn'][3]

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

    def _build_ik(self, path, ik_path):

        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
        env = openravepy.Environment()
        plugin = openravepy.RaveCreateModule(env, "urdf")

        with env:
            name = plugin.SendCommand('load {} {}'.format(path, ik_path))
            robot = env.GetRobot(name)

        robot.SetActiveManipulator('arm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                robot, iktype=openravepy.IkParameterization.Type.Transform6D
        )
        if not ikmodel.load():
            ikmodel.autogenerate()
        return ikmodel

    def _move_to(self, pos, orn, cc=True):
        
        # Convert to pose in robot base frame
        orn = self.kinematics['orn'][18] if orn is None else orn
        pos, orn = math_util.get_relative_pose((pos, orn), self.pose)

        indices = [5, 10, 11, 12, 13, 15, 18]

        # openrave quaternion uses wxyz convention
        ik_solution = OpenRaveEngine.solve_ik(
            self._ik_model, pos, orn[[3,0,1,2]], 
            math_util.vec(self.joint_states['pos'])[indices])

        specs = self.joint_specs

        self.joint_states = (
            indices, ik_solution, 'position',
            dict(forces=math_util.vec(specs['max_force'])[indices],
                 positionGains=(.03,) * self._dof,
                 velocityGains=(1.,) * self._dof))
