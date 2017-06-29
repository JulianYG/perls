
from .body import Tool
from ..utils import math_util


class PrismaticGripper(Tool):

    def __init__(self, tid, engine, path, pos, orn, left, right):
        """
        Initialization.
        :param path: asset file path
        :param pos: initial position, vec3 float cartesian
        :param orn: initial orientation, vec4 quat
        :param left: left tip link index, as specified in asset file
        :param right: right tip link index
        """
        Tool.__init__(self, tid, engine, path, pos, orn, False)

        # Need to be specified in child class
        self._left_finger_idx = left
        self._right_finger_idx = right
        self.reset()

    @property
    def left_finger(self):
        return self._left_finger_idx

    @property
    def right_finger(self):
        return self._right_finger_idx

    @property
    def tool_pos(self):
        """
        Get the position of the gripper, based on 
        the average of left and right tips/fingers 
        positions to achieve accuracy
        :return: vec3 float in Cartesian
        """
        left_finger_pos = self.kinematics['pos'][self._left_finger_idx]
        right_finger_pos = self.kinematics['pos'][self._right_finger_idx]
        return (left_finger_pos + right_finger_pos) / 2. + self._tip_offset

    @property
    def tool_orn(self):
        """
        Get the orientation of the gripper. This is semantic
        :return: vec4 float quaternion in Cartesian
        """
        return self.orn

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the gripper to given position. Use left finger as reference.
        :param pos: vec3 float in cartesian space
        :return: None
        """
        if self.fix:
            print('Cannot move attached gripper.')
            return
        # Need some transformation
        base_pos = self.position_transform(pos, self.tool_orn)

        # Note here it only cares about the position,
        # thus not solving using constraints
        self.pos = base_pos

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the gripper to given orientation
        :param orn: vec4 float in quaternion form
        :return: None
        """
        self.orn = orn

    def position_transform(self, pos, orn):
        """
        Helper function to convert position between 
        fingers of gripper to position on gripper 
        base link
        :param pos: vec3 float cartesian world frame
        :param orn: vec4 float quaternion world frame
        :return: vec3 float cartesian world frame
        """
        # Get Center of Mass (CoM) of averaging
        # left/right gripper fingers
        translation = (
            self.kinematics['pos'][self._left_finger_idx] +
            self.kinematics['pos'][self._right_finger_idx]) / 2. -\
            self.pos
        # Since desired frame is aligned with base frame...
        rotation = math_util.get_rotation_from_quaternion(orn)
        base_pos = pos - rotation.dot(translation)
        return base_pos

    ###
    #  High level functionalities
    def reset(self):
        self._engine.update()

    def reach(self, pos, orn=None):
        """
        Reach to given pose
        :param pos: vec3 float cartesian
        :param orn: vec4 float quaternion
        :return: delta between target and actual pose
        """
        if orn is None:
            orn = self.tool_orn
        base_pos = self.position_transform(pos, orn)
        # Use constraint to move gripper for simulation,
        # to avoid boundary mixing during collision
        self.track(base_pos, orn, 200)
        orn_delta = self.tool_orn - orn
        pos_delta = self.tool_pos - pos
        return pos_delta, orn_delta

    def grasp(self, slide):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        raise NotImplementedError('Method <grasp> not implemented for gripper. '
                                  'Method is gripper-specific')

