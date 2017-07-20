
from .body import Tool, Body
from ..utils import math_util
from ..utils.io_util import (FONT,
                             loginfo,
                             logerr)


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
        super(PrismaticGripper, self).__init__(tid, engine, path, pos, orn, False)

        # Need to be specified in child class
        self._left_finger_idx = left
        self._right_finger_idx = right
        self._max_force = 200
        self.reset()

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: interger tool id
        """
        return 'g{}'.format(self._tool_id)

    @Body.fix.getter
    def fix(self):
        """
        Definition of fixed is a little bit different here.
        Fix is relative to the robot arm end effector.
        Gripper, as a tool, cannot be 'fixed' in the sense
        that it cannot move at all.
        """
        return False if -1 in self.attach_children else True

    @property
    def left_finger(self):
        return self._left_finger_idx

    @property
    def right_finger(self):
        return self._right_finger_idx

    @property
    def traction(self):
        return self._max_force

    @property
    def pose(self):
        return self.pos, self.orn

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

    @traction.setter
    def traction(self, f):
        self._max_force = f

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the gripper to given position. Use left finger as reference.
        :param pos: vec3 float in cartesian space
        :return: None
        """
        if self.fix:
            logerr('Cannot move attached gripper.',
                   FONT.model)
            return

        # Note here it only cares about the position,
        # thus not solving using constraints
        self.track(pos, self.tool_orn, self._max_force)

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
        rotation = math_util.quat2mat(orn)
        base_pos = pos - rotation.dot(translation)
        return base_pos

    ###
    #  High level functionality
    def reset(self):
        """
        Release gripper for reset
        :return: None
        """
        pos, orn, _ = self._init_state
        if not self.fix:
            self.track(pos, orn, self._max_force)
        self.grasp(0)
        self._engine.hold()

    def hang(self):
        """
        Hang the gripper in the world for control
        :return: None
        """
        self.attach_children = (
            -1, -1, -1, 'fixed', [0., 0., 0.],
            [0., 0., 0.], self.pos, None, self.orn)

    def reach(self, pos=None, orn=None, ftype='abs'):
        """
        Reach to given pose approximately
        :param pos: vec3 float cartesian at base
        :param orn: vec4 float quaternion
        :return: delta between target and actual pose
        """
        fpos, forn = super(PrismaticGripper, self).reach(pos, orn, ftype)

        pos_delta = math_util.zero_vec(3)
        forn = self.tool_orn if forn is None else forn

        # Use constraint to move gripper for simulation,
        # to avoid boundary mixing during collision
        self.track(fpos, forn, self._max_force)

        orn_delta = math_util.quat_diff(self.tool_orn, forn)
        if fpos is not None:
            pos_delta = self.tool_pos - fpos

        return pos_delta, orn_delta

    def pinpoint(self, pos, orn, ftype='abs'):
        """
        Accurately reach to given pose
        :param pos: vec3 float cartesian at finger tip
        :param orn: vec4 float quaternion
        :return: None
        """
        fpos, forn = super(PrismaticGripper, self).pinpoint(pos, orn, ftype)
        self.tool_pos = fpos
        self.tool_orn = forn

    def grasp(self, slide):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        raise NotImplementedError(
            'Method <grasp> not implemented for gripper. '
            'Method is gripper-specific')

