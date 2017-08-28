
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
    def type(self):
        """
        Get the type of the body.
        :return: string
        """
        return 'gripper'

    @property
    def close_grip(self):
        """
        Status of gripper, whether it's closed
        :return: boolean
        """
        return self._close_grip

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
        left_finger_pos = self.kinematics['abs_frame_pos'][self._left_finger_idx]
        right_finger_pos = self.kinematics['abs_frame_pos'][self._right_finger_idx]
        return (left_finger_pos + right_finger_pos) / 2. + \
            math_util.quat2mat(self.tool_orn).dot(self._tip_offset)

    @property
    def tool_orn(self):
        """
        Get the orientation of the gripper. This is semantic
        :return: vec4 float quaternion in Cartesian
        """
        return self.orn

    @property
    def tolerance(self):
        """
        Get the error margin of tool tip position due to  
        rotation. 
        :return: float scalar distance
        """
        return math_util.rms(self.tool_pos - self.pos)

    @traction.setter
    def traction(self, f):
        self._max_force = f

    @tool_pos.setter
    def tool_pos(self, pos_iter):
        """
        Set the gripper to given position. Use left finger as reference.
        :param pos: vec3 float in cartesian space
        :return: None
        """
        if self.fix:
            logerr('Cannot move attached gripper.',
                   FONT.model)
            return
        pos = self.position_transform(pos_iter[0], self.tool_orn)[0]

        # Note here it only cares about the position,
        # thus not solving using constraints
        self.track(pos, self.tool_orn, self._max_force)

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the gripper to given orientation
        :param orn: vec4 float in quaternion form or 
        vec3 float in euler radian form
        :return: None
        """
        if len(orn) == 3:
            orn = math_util.euler2quat(orn)
        self.track(self.pos, orn, self._max_force)

    def position_transform(self, pos, orn):
        """
        Helper function to convert position between 
        fingers of gripper to position on gripper 
        base link
        :param pos: vec3 float cartesian world frame
        :param orn: vec4 float quaternion world frame
        :return: transformed pose (pos, orn) in world frame
        """
        # Get Center of Mass (CoM) of averaging
        # left/right gripper fingers

        transform = math_util.pose2mat(
            math_util.get_relative_pose(
                (self.pos, self.orn), (self.tool_pos, orn))
            )
        frame = math_util.pose2mat((pos, orn)).dot(transform)
        return math_util.mat2pose(frame)

    ###
    #  High level functionality
    def reset(self):
        """
        Release gripper for reset
        :return: None
        """
        pos, orn, _ = self._init_state
        if not self.fix:
            # pos, orn = self.position_transform(pos, orn)
            self.track(pos, orn, self._max_force)
        self.grasp(0)

    def hang(self):
        """
        Hang the gripper in the world for control
        :return: None
        """
        self.attach_children = (
            -1, -1, -1, 'fixed', [0., 0., 0.],
            [0., 0., 0.], self.pos, None, self.orn)

    def pinpoint(self, pos, orn, ftype='abs'):
        """
        Accurately reach to given pose
        :param pos: vec3 float cartesian at finger tip
        :param orn: vec4 float quaternion
        :return: delta between target and actual pose
        """
        fpos, forn = super(PrismaticGripper, self).pinpoint(pos, orn, ftype)
        self.tool_pos = fpos
        self.tool_orn = forn

        pos_delta = self.tool_pos - pos
        orn_delta = math_util.quat_diff(self.tool_orn, orn)
        return pos_delta, orn_delta

    def grasp(self, slide):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        raise NotImplementedError(
            'Method <grasp> not implemented for gripper. '
            'Method is gripper-specific')
