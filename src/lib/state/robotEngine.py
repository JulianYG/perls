#!/usr/bin/env python

from .stateEngine import RealStateEngine

import sys, os 
sys.path.append(os.path.abspath(os.path.join(__file__, '../../pyrobots')))

import rospy
import moveit_commander

# from sawyer import SawyerArm

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class InteraEngine(RealStateEngine):

    def __init__(self, e_id, max_run_time, plan=True):

        super(InteraEngine, self).__init__(e_id, max_run_time)

        self._uids = dict()

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = SawyerArm(plan)
        # self._virtual_scene = moveit_commander.PlanningSceneInterface()
        # self._group = moveit_commander.MoveGroupCommander("right_arm")

    @property
    def version(self):
        return dict(SDKVersion=iif.settings.SDK_VERSION,
                    SDK2Gripper=iif.settings.VERSIONS_SDK2GRIPPER,
                    SDK2Robot=iif.settings.VERSIONS_SDK2ROBOT)

    @property
    def info(self):

        return self._info

    def configure(self, configs):
        """
        Configure the real state of robot
        :param configs: configuration
        :return: None
        """
        pass

    def load_asset(self, file_path, pos, orn, fixed):
        """
        Load the mesh asset of an entity (body) into the world.
        File typically given in urdf/sdf/xml format.
        :param file_path: absolute or relative path of asset file
        :param pos: position vec3 float cartesian
        :param orn: orientation vec4 float quaternion tuple
        :param fixed: boolean indicating whether the body should
        be fixed to given pose when loaded.
        :return: integer of body unique id, body link indices,
        body joint indices
        """
        self._uids[0] = self._robot

    def get_body_scene_position(self, uid):
        """
        In real world, body position is given by ar_tracker.
        :param uid: integer body unique id
        :return: position vec3 float cartesian
        """
        pass

    def get_body_scene_orientation(self, uid, otype='quat'):
        """
        In real world, body orientation is given by ar_tracker.
        :param uid: integer body unique id
        :param otype: orientation type, can be specified as
        'quat' for quaternion (vec4 float),
        'deg' for euler degrees (vec3 float), or
        'rad' for euler radians (vec3 float).
        :return: orientation vector in given otype
        """
        pass

    def get_body_camera_position(self, uid, camera_pos, camera_orn):
        """
        Get body position in the camera frame
        :param uid: integer body unique id
        :param camera_pos: vec3 float cartesian camera
        position in the world frame.
        :param camera_orn: vec4 float quaternion camera
        orientation in the world frame.
        :return: position vec3 float cartesian
        """
        pass

    def get_body_camera_orientation(
            self, uid, camera_pos, camera_orn, otype):
        """
        Get body orientation in the camera frame
        :param uid: integer body unique id
        :param camera_pos: vec3 float cartesian camera
        position in the world frame.
        :param camera_orn: vec4 float quaternion camera
        orientation in the world frame.
        :param otype: orientation type, can be specified as
        'quat' for quaternion (vec4 float),
        'deg' for euler degrees (vec3 float), or
        'rad' for euler radians (vec3 float).
        :return: orientation vector in given otype
        """
        pass

    def get_body_relative_pose(self, uid, frame_pos, frame_orn):
        """
        Get the body's position and orientation in the
        specified coordinate system. In a real world, this
        should have something to do with the camera.
        :param uid: body's unique integer id
        :param frame_pos: origin of given frame (vec3 float
         cartesian), relative to the world frame
        :param frame_orn: orientation of given frame (vec4
        float quaternion), relative to the world frame
        :return: (pos, orn) of querying object in given frame
        """
        pass

    def get_body_name(self, uid):
        """
        Get the name string of the body
        :param uid: integer body unique id
        :return: string of name
        """
        pass

    def get_link_name(self, uid, lid):
        """
        Get the name string of given link
        :param uid: integer body unique id
        :param lid: integer link id of body
        :return: name string of link on body
        """
        pass

    def get_body_linear_velocity(self, uid):
        """
        Get the linear velocity of body in world frame.
        Ar_tracker
        :param uid: body unique id
        :return: vec3 float cartesian linear velocity
        """
        pass

    def set_body_linear_velocity(self, uid, vel):
        """
        Cannot set the body linear velocity in real world
        """
        pass

    def get_body_angular_velocity(self, uid):
        """
        Get the angular velocity of body in world frame.
        Ar_tracker
        :param uid: body unique id
        :return: vec3 float cartesian angular velocity
        in radian
        """
        pass

    def set_body_angular_velocity(self, uid, vel):
        """
        Cannot set the body angular velocity in real world
        """
        pass

    def get_body_link_state(self, uid, lid):
        """
        Get the state of given link on given body
        :param uid: integer body unique id
        :param lid: integer link id on body
        :return: a tuple of
        link world position (vec3 float cartesian),
        link world orientation (vec4 float quaternion),
        local position offset of CoM frame,
        local orientation offset of CoM frame,
        world position of asset file link frame,
        world orientation of asset file link frame,
        cartesian link world linear velocity,
        radian link world angular velocity.
        """
        pass

    def get_body_joint_info(self, uid, jid):
        """
        Get the information of body joint
        :param uid: integer unique body id
        :param jid: integer joint id on body
        :return: a tuple of
        joint index,
        joint name string in asset file,
        joint type string ('fixed', 'revolute',
        'prismatic', 'spherical', 'planar', 'gear',
        'point2point'),
        the first position index in the positional
        state variables for this body,
        the first velocity index in the velocity
        state variables for this body,
        joint damping value,
        joint friction coefficient,
        joint lower limit,
        joint upper limit,
        maximum force joint can sustain,
        maximum velocity joint can sustain,
        name string of associated link.
        """
        pass

    def get_body_joint_state(self, uid, jid):
        """
        Get the current state of body joint
        :param uid: unique integer body id
        :param jid: integer joint id on body
        :return: a tuple of
        joint position float (radian),
        joint velocity float (rad/s),
        wrench on joint (vec6 float, force3 + torque3),
        applied motor torque on joint float.
        """
        pass

    def set_body_joint_state(self, uid, jids, vals, ctype, kwargs):
        """
        Set the body joint state
        :param uid: integer body unique id
        :param jids: list of joint indices on body
        :param vals: list of values to set on joints.
        Length must align with that of joint indices.
        :param ctype: string control type, choose among
        'position', 'velocity', and 'torque'
        :param kwargs: other key word arguments to the function,
        such as 'reset', 'positionGain', 'velocityGain', 'maxForce', etc.
        :return: None
        """
        pass

    def disable_body_joint_motors(self, uid, jids):
        """
        Disable the motors. This is desired for the case
        of direct torque control, where motors need to be
        disabled.
        :param uid: body unique id of the arm
        :param jids: indices of joints to be disabled
        :return: None
        """
        self._robot_enable.disable()

    def enable_body_joint_motors(self, uid, jids, forces):
        """
        Enable the motors. This is desired after using
        direct torque control, where motors need to be
        re-activated for other modes of control.
        :param uid: body unique id of the arm
        :param jids: indices of joints to be enabled
        :param forces: force arrays to set all joints
        :return: None
        """
        self._robot_enable.enable()

    def get_body_dynamics(self, uid, lid):
        """
        Get the body dynamics info
        :param uid: integer body unique id
        :param lid: integer link id on body
        :return: a tuple of
        mass, float kg
        friction coefficient, float
        """
        pass

    def start_engine(self, frame):
        """
        Start the render
        :param frame: string of frame type,
        :return: 0 if success, -1 if failed
        """
        pass

    def hold(self, max_steps=1000):
        """
        Leave the world alone for given
        period of time
        :param max_steps: integer number of steps
        to pause external actions in world
        :return: None
        """
        pass

    def step(self, elapsed_time, step_size):
        """
        Simulate for one step
        :param elapsed_time: integer elapsed time
        since controller started running, used for
        time out in real time simulation
        :return: None
        """
        pass

    def stop(self):
        """
        Stop the render and clean up.
        :return: None
        """
        pass

    ################################################
    # Methods that cannot be implemented in real world

    def set_body_scene_pose(self, uid, pos, orn):
        """
        Cannot set body pose in real world through code
        """
        return NotImplemented

    def get_body_visual_shape(self, uid):
        """
        Cannot get the visual shape of body
        in real world
        """
        return NotImplemented

    def set_body_visual_color(self, uid, qid, color, spec=False):
        """
        Cannot set the visual color of body
        in real world
        """
        return NotImplemented

    def set_body_texture(self, uid, qid, texture):
        """
        Cannot set body texture in real world
        """
        return NotImplemented

    def set_body_visual_shape(self, uid, qid, shape):
        """
        Cannot set body shape in real world
        """
        return NotImplemented

    def change_loaded_texture(self, texture_id, pixels, w, h):
        """
        Cannot change the loaded texture in real world
        """
        return NotImplemented

    def set_body_dynamics(self, uid, lid, info):
        """
        Cannot change dynamics of body based on given info
        in real world
        """
        return NotImplemented

    def get_body_bounding_box(self, uid, lid):
        """
        Cannot get the bounding box of given body in real world
        """
        return NotImplemented

    def get_body_contact_info(self, uid, lid):
        """
        Cannot get body contact info in real world
        """
        return NotImplemented

    def get_body_surroundings(self, uidA, lidA, uidB, lidB, dist):
        """
        Cannot get the neighbors of given body and link
        in real world
        """
        return NotImplemented

    def add_body_text_marker(self, text, pos, font_size, color,
                             uid, lid, time):
        """
        Cannot mark text string on given body in real world
        """
        return NotImplemented

    def add_body_line_marker(self, posA, posB, color, width,
                             time, uid, lid):
        """
        Cannot mark line on given body in real world
        """
        return NotImplemented

    def remove_body_text_marker(self, marker_id):
        """
        Cannot remove the text by given id in real world
        """
        return NotImplemented

    def apply_force_to_body(self, uid, lid, force, pos, ref):
        """
        Cannot apply external force to given body/link
        in real world
        """
        return NotImplemented

    def apply_torque_to_body(self, uid, lid, torque, ref):
        """
        Cannot apply external torque to given body/link in
        real world
        """
        return NotImplemented

    def get_body_attachment(self, uid):
        """
        Cannot get the attached bodies on given body in real world
        """
        return NotImplemented

    def set_body_attachment(self, parent_uid, parent_lid,
                            child_uid, child_lid,
                            jtype='fixed',
                            jaxis=(0., 0., 0.),
                            parent_pos=(0., 0., 0.),
                            child_pos=(0., 0., 0.),
                            **kwargs):
        """
        Cannot attach given child body to given parent body in
        real world
        """
        return NotImplemented

    def remove_body_attachment(self, cid):
        """
        Cannot remove the given attachment in ral world
        """
        return NotImplemented

    def move_body(self, cid, pos, orn, force):
        """
        Cannot move a fixed body to given pose with given force
        in real world
        """
        return NotImplemented

    def delete_body(self, uid):
        """
        Cannot remove given body from environment in real world
        """
        return NotImplemented


class ROSEngine(RealStateEngine):

    pass


class MoveitEngine(RealStateEngine):

    pass


