#!/usr/bin/env python


import rospy
# import rosparam
import intera_interface as iif


class SawyerArm(object):

    def __init__(self):

        if rospy.get_name() == '/unnamed':
            rospy.init_node('robot')

        self._head = iif.Head()
        self._display = iif.HeadDisplay()
        self._lights = iif.Lights()

        self._limb = iif.Limb()
        self._joints = self._limb.joint_names()

        # self._navigator = iif.Navigator()

        try:
            self._gripper = iif.Gripper()
            self._has_gripper = True
        except:
            self._has_gripper = False

        self._robot_enable = iif.RobotEnable(True)

        self._params = iif.RobotParams()

    @property
    def version(self):
        return dict(SDKVersion=iif.settings.SDK_VERSION,
                    SDK2Gripper=iif.settings.VERSIONS_SDK2GRIPPER,
                    SDK2Robot=iif.settings.VERSIONS_SDK2ROBOT)

    @property
    def name(self):
        return self._params.get_robot_name()

    @property
    def tool_pose(self):
        """
        Get the pose of the end effector.
        :return: vec3 float, vec4 float pos, orn tuple
        """
        pose = self._limb.endpoint_pose().values()
        return (pose[0].x, pose[0].y, pose[0].z), (pose[1].x, pose[1].y, pose[1].z, pose[1].w)

    @property
    def v(self):
        return self._limb.endpoint_velocity()['linear']

    @property
    def omega(self):
        return self._limb.endpoint_velocity()['angular']

    @property
    def wrench(self):
        eef_effort = self._limb.endpoint_effort()
        force = list(eef_effort['force'])
        torque = list(eef_effort['torque'])
        return tuple(force + torque)

    @property
    def joint_states(self):
        """
        Get the current state of robot joints
        :return: a list of dictionaries:
        string joint name,
        joint position float (radian),
        joint velocity float (rad/s),
        wrench on joint (vec6 float, force3 + torque3),
        applied motor torque on joint float,
        in order from base to end effector.
        """
        names = self._limb.joint_names()
        positions = self._limb.joint_angles()
        velocities = self._limb.joint_velocities()
        efforts = self._limb.joint_efforts()

        return dict(name=names,
                    position=[positions[n] for n in names],
                    velocity=[velocities[n] for n in names],
                    effort=[efforts[n] for n in names])

    @property
    def info(self):

        assembly_names = self._params.get_robot_assemblies()
        camera_info = self._params.get_camera_details()

        return assembly_names, camera_info

    def configure(self, configs):
        """
        Configure the real state of robot
        :param configs: configuration
        :return: None
        """
        pass

    @property
    def head_pan(self):
        """
        Get current pan angle of the head
        :return: float radian
        """
        return self._head.pan()

    @head_pan.setter
    def head_pan(self, (angle, speed, rel)):
        """
        Pan at given speed to desired angle
        :param angle: float radian
        :param speed: float speed, 0 - 1
        :param rel: if True, pan wrt base frame
        :return: None
        """
        self._head.set_pan(angle, speed=speed,
                           active_cancellation=rel)

    def get_link_name(self, uid, lid):
        """
        Get the name string of given link
        :param uid: integer body unique id
        :param lid: integer link id of body
        :return: name string of link on body
        """
        pass

    def get_link_state(self, lid):
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

    def get_joint_info(self, jid):
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
        joint_names = self._params.get_joint_names('right')
        return joint_names

    @joint_states.setter
    def joint_states(self, (jids, vals, ctype, kwargs)):
        """
        Set the body joint state
        :param uid: integer body unique id
        :param jids: list of joint indices on body
        :param vals: list of values to set on joints.
        Length must align with that of joint indices.
        :param ctype: string control type, choose among
        'position', 'velocity', and 'torque'
        :param kwargs: other key word arguments to the function,
        such as 'max_speed', 'timeout', 'threshold', 'test', etc.
        :return: None
        """
        assert len(jids) == len(vals), 'Joint indices and values mismatch'
        # Safety call
        self._limb.set_joint_position_speed(kwargs.get('max_speed', 0.3))

        del kwargs['max_speed']

        command = {self._joints[i]: vals[i] for i in range(len(vals))}
        if ctype == 'position':
            # blocking call
            self._limb.move_to_joint_positions(command, **kwargs)
        elif ctype == 'velocity':
            self._limb.set_joint_velocities(command)
        elif ctype == 'torque':
            self._limb.set_joint_torques(command)
        else:
            self._params.log_message('Cannot recognize control type', 'WARN')    

    def show_image(self, image_path, rate=1.0):

        self._display.display_image(image_path, display_rate=rate)

    @property
    def light(self):
        """
        Get the info (names) of all available lights
        :return: A dictionary where keys are light name strings,
        and values are their boolean on status
        """
        return {name: self._lights.get_light_state(name)
                for name in self._lights.list_all_lights()}

    @light.setter
    def light(self, (name, on)):
        """
        Set the status of given light
        :param name: string name of the light
        :param on: boolean True for on, False for off
        :return: True if light state is set, False if not
        """
        self._lights.set_light_state(name, on)

    @property
    def gripper_force(self):
        """
        Returns the force sensed by the gripper in estimated Newtons.
        :return: float current force value in N-m
        """
        if self._has_gripper:
            return self._gripper.get_force()

    # TODO: Cannot find signal 'holding_force_n' in this IO Device.
    @gripper_force.setter
    def gripper_force(self, force):
        """
        Set the holding force on the gripper for successful grasp
        :param force:
        :return:
        """
        if self._has_gripper:
            if not self._gripper.set_holding_force(force):
                self._params.log_message('Unable to set holding force'
                                         'for the gripper.', 'WARN')

    def set_grasp_weight(self, weight):
        """
        Set the weight of object the gripper is grasping
        :param weight: float in kg
        :return: True if successful, False if failure
        """
        return self._gripper.set_object_weight(weight)

    def grasp(self, slide=-1):
        """
        Set the gripper grasping
        :param slide:
        :return: None
        """
        if self._has_gripper:
            if slide > -1:
                # Use slide grasp in this case
                scaled_pos = slide * self._gripper.MAX_POSITION
                self._gripper.set_position(scaled_pos)
            else:
                # Just toggle the state
                curr_pos = self._gripper.get_position() / self._gripper.MAX_POSITION
                if self._gripper.is_gripping() or curr_pos < 0.5:
                    self._gripper.open()
                else:
                    self._gripper.close()

    def pause(self):
        """
        Pause the action and disable all motors.
        Call <start> to resume.
        :return: None
        """
        if self._has_gripper:
            self._gripper.stop()
        self._robot_enable.disable()

    def start(self):
        """
        Start the robot
        :return: None
        """
        self._robot_enable.enable()
        if self._has_gripper:
            self._gripper.start()
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

    def reset(self):
        """
        Reset the robot and move to rest pose
        :return: None
        """
        if self._robot_enable.state().error:
            self._robot_enable.reset()
        if self._has_gripper:
            self._gripper.reboot()
            self._gripper.calibrate()
        self._limb.move_to_neutral()
        self._gripper.start()

    def stop(self):
        """
        Act as E-stop. Must reset to clear
        the stopped state
        :return: None
        """
        self._robot_enable.stop()
        if self._has_gripper:
            self._gripper.stop()

    def shutdown(self):
        rospy.signal_shutdown('Safely shut down Sawyer.')
