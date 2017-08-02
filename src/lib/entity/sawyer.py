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

        # Active joints indices: 5, 10, 11, 12, 13, 15, 18
        # Note the urdf has been modified and removed right_hand link, joint
        # self._rest_pose = math_util.vec((0, 0, 0, 0, 0,
        #                    0, 0, 0, 0, 0,
        #                    -1.18, 0.00, 2.18, 0.00,
        #                    0, 0.57, 0, 0, 3.3161))
        # Before pose was [4]
        self._rest_pose = math_util.vec((0, -1.18, 0.00,
                                         2.18, 0.00, 0.57, 3.3161))
        self._dof = 7
        # self._active_dof = [5, 10, 11, 12, 13, 15, 18]
        self._active_dof = range(7)
        # self._active_dof = [1,2,3,4,5,6,7]

        self.reset()

    @property
    def active_joints(self):
        """
        Return the joint indices that are active (settable)
        :return: a list of indices integers
        """
        return self._active_dof

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos - self.kinematics['pos'][self._end_idx]) * 2

    def _build_ik(self, path_root):

        bullet_model_path = io_util.pjoin(path_root, 'sawyer_arm.urdf')#'sawyer.urdf')
        ikfast_model_path = io_util.pjoin(path_root, 'sawyer_arm_ikfast.urdf')
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
        print(robot.GetActiveDOFValues(), robot.GetActiveDOF(), robot.GetActiveJointIndices(), robot.GetActiveManipulator())
        return bullet_model_path, ikmodel, robot, [1, 2, 3, 4, 5, 6, 7]
