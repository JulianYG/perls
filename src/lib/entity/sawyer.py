from .arm import Arm
from ..utils import math_util, io_util

import openravepy


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

        # Note the urdf has been modified and removed right_hand link & joint
        # self._rest_pose = math_util.vec(
            # (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.18, 0.00, 2.18, 0.00,
             # 0, 0.57, 0, 0, 3.3161))
        # Before pose was [4]
        self._rest_pose = math_util.vec((0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161))

        self._dof = 7
        self.reset()

    @property
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        return [5, 10, 11, 12, 13, 15, 18]

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos - self.kinematics['pos'][self._end_idx]) * 2

    def _build_ik(self, path_root):

        model_path = io_util.pjoin(path_root, 'sawyer.urdf')
        base_path = io_util.pjoin(path_root, 'sawyer.srdf')

        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
        env = openravepy.Environment()
        plugin = openravepy.RaveCreateModule(env, "urdf")
        
        with env:
            name = plugin.SendCommand('load {} {}'.format(model_path, base_path))
            robot = env.GetRobot(name)

        robot.SetActiveManipulator('gripper')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                robot, iktype=openravepy.IkParameterization.Type.Transform6D
        )
        ikmodel.load()
        return model_path, ikmodel, robot, range(7)
