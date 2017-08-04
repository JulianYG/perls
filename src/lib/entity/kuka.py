import openravepy

from .arm import Arm
from .WSG50Gripper import WSG50Gripper

from ..utils import math_util, io_util


class Kuka(Arm):

    def __init__(self, tool_id, engine,
                 path=None,
                 pos=(0., 0., 0.67),
                 orn=(0., 0., 0., 1.),
                 collision_checking=True,
                 gripper=None):
        path = path or '../../data/kuka_iiwa'
        super(Kuka, self).__init__(
            tool_id, engine, path, pos, orn, collision_checking, gripper)
        self._tip_offset = math_util.vec([0., 0., 0.045])
        self._rest_pose = (0., 0., 0., 1.570793, 0., -1.04719755, 0.)
        self._active_dof = self._joints
        self.reset()

    def _build_ik(self, path_root):

        bullet_model_path = io_util.pjoin(path_root, 'model_vr_limits.urdf')
        ikfast_model_path = io_util.pjoin(path_root, 'kuka_arm.robot.xml')

        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
        env = openravepy.Environment()
        env.Load(ikfast_model_path) # load a scene

        robot = env.GetRobots()[0] # get the first robot
        robot.SetActiveManipulator('arm') # set the manipulator

        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot, iktype=openravepy.IkParameterization.Type.Transform6D
        )

        ikmodel.load()

        return bullet_model_path, ikmodel, robot, range(7)
