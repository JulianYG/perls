#!/usr/bin/env python

import sys, time
from threading import Thread, Event

try:
    import moveit_commander
    from moveit_msgs.msg import (
        Grasp,
        GripperTranslation,
        DisplayTrajectory
    )
except ImportError:
    print('Problems with moveit!')

try:
    import rospy
    import intera_interface as iif
    from intera_interface import CHECK_VERSION

    from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
    )

    from intera_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        SolvePositionFK,
        SolvePositionFKRequest
    )

    from intera_core_msgs.msg import JointCommand

    from std_msgs.msg import Header
    from sensor_msgs.msg import JointState
except ImportError:
    pass


KINECT_DEPTH_SHIFT = -22.54013555237548
GRIPPER_SHIFT = 0.0251
LENGTH = 0.133
FINGER_OFFSET = 0.066


class Commander(Thread):

    def __init__(self, freq, func, *args):

        super(Commander, self).__init__()
        self._func = func
        self._args = args
        self._done = Event()
        self.daemon = True
        self._pause = False
        self._rate = rospy.Rate(freq)

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self._done.set()

    def pause(self):
        self._pause = True

    def resume(self):
        self._pause = False

    def run(self):
        while not self._done.is_set():
            if not self._pause:
                self._func(*self._args)
                self._rate.sleep()


class SawyerArm(object):


    def __init__(self, safe=True, motion_planning=False):
        """
        Initialize set of wrappers
        """
        # rospy.init_node('sawyer')
        assert rospy.get_name() != '/unnamed', 'Must init node!'
        start = time.time()
        print('Initializing robot...')
        self.motion_planning = motion_planning
        self._head = iif.Head()
        self._display = iif.HeadDisplay()
        self._lights = iif.Lights()

        self._limb = iif.Limb()
        self._joints = self._limb.joint_names()
        
        # self._navigator = iif.Navigator()
        self.cmd = []

        self._safenet = rospy.Publisher(
            '/safeNet/joint_command',
            JointCommand, queue_size=1)

        self._command_msg = JointCommand()
        self._command_msg.names = self._joints

        self._commanders = dict(velocity=None, torque=None)

        try:
            self._gripper = iif.Gripper()
            self._has_gripper = True
        except:
            self._has_gripper = False

        self._robot_enable = iif.RobotEnable(True)

        self._params = iif.RobotParams()

        # Initialize motion planning part
        if motion_planning:
            moveit_commander.roscpp_initialize(sys.argv)
            self._scene = moveit_commander.PlanningSceneInterface()
            self._group = moveit_commander.MoveGroupCommander("right_arm")
        self._safe = safe

        print('Initialization finished after {} seconds'.format(time.time() - start))

    @property
    def version(self):
        """
        List current versions of wrapped SDK,
        gripper, and robot
        :return: dictionary of version strings
        """
        return dict(SDKVersion=iif.settings.SDK_VERSION,
                    SDK2Gripper=iif.settings.VERSIONS_SDK2GRIPPER,
                    SDK2Robot=iif.settings.VERSIONS_SDK2ROBOT)

    @property
    def name(self):
        """
        Get the name of the robot
        :return: 'Sawyer'
        """
        return self._params.get_robot_name()

    @property
    def tool_pose(self):
        """
        Get the pose of the end effector.
        :return: vec3 float, vec4 float pos, orn tuple
        """
        pose = self._limb.endpoint_pose().values()
        return (pose[0].x, pose[0].y, pose[0].z), \
               (pose[1].x, pose[1].y, pose[1].z, pose[1].w)

    @property
    def v(self):
        """
        Get the current linear velocity of end effector
        :return: vec3 float cartesian m/s
        """
        return self._limb.endpoint_velocity()['linear']

    @property
    def omega(self):
        """
        Get the current angular velocity of end effector
        :return: vec3 float euler radian /s
        """
        return self._limb.endpoint_velocity()['angular']

    @property
    def wrench(self):
        """
        Get the wrent (mx, my, mz, fx, fy, fz) on end effector
        :return: vec6 float tuple
        """
        eef_effort = self._limb.endpoint_effort()
        force = list(eef_effort['force'])
        torque = list(eef_effort['torque'])
        return tuple(force + torque)

    @property
    def joint_positions(self):
        """
        Get the joint positions of the body/tool.
        :return: a list of joint positions in radian, which
        is ordered by indices from small to large.
        Typically the order goes from base to end effector.
        """
        positions = self._limb.joint_angles()
        return [positions[n] for n in self._joints]

    @property
    def joint_velocities(self):
        """
        Get the joint velocities of the body/tool.
        :return: a list of joint velocities in radian/s, which
        is ordered by indices from small to large.
        Typically the order goes from base to end effector.
        """
        velocities = self._limb.joint_velocities()
        return [velocities[n] for n in self._joints]

    @property
    def joint_torques(self):
        """
        Get the joint torques of the body/tool. This is
        the torque applied by the actuators during last
        simulation step.
        :return: a list of joint torques in radian/s, which
        is ordered by indices from small to large.
        Typically the order goes from base to end effector.
        """
        torques = self._limb.joint_efforts()
        return [torques[n] for n in self._joints]

    @property
    def info(self):
        """
        Get informations about parameters of the robot
        :return: names and information
        """
        assembly_names = self._params.get_robot_assemblies()
        camera_info = self._params.get_camera_details()

        return assembly_names, camera_info

    def configure(self, configs):
        """
        Configure the real state of robot
        :param configs: configuration
        :return: None
        """
        return NotImplemented

    @property
    def head_pan(self):
        """
        Get current pan angle of the head
        :return: float radian
        """
        return self._head.pan()

    @head_pan.setter
    def head_pan(self, angle_speed_rel):
        """
        Pan at given speed to desired angle
        :param angle: float radian
        :param speed: float speed, 0 - 1
        :param rel: if True, pan wrt base frame
        :return: None
        """
        angle, speed, rel = angle_speed_rel
        self._head.set_pan(angle, speed=speed,
                           active_cancellation=rel)

    def get_link_name(self, uid, lid):
        """
        Get the name string of given link
        :param uid: integer body unique id
        :param lid: integer link id of body
        :return: name string of link on body
        """
        return NotImplemented

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
        return NotImplemented

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

    @joint_positions.setter
    def joint_positions(self, value):
        """
        Set joint positions according to given values list. Note
        that the length of the list must match that of the
        joint indices, and if one can skip certain joints by
        setting those values to None.
        Not suggested to use since it has no safety precautions
        :param value: A list of length DOF, 
        or a tuple where the first element is the joint velocities list,
        and the second element is the keyword arguments dictionary.
        :return: None
        """
        if isinstance(value, tuple) and len(value) == 2:
            vals, kwargs = value
        else:
            vals, kwargs = value, {}

        # Safety call
        self._limb.set_joint_position_speed(kwargs.get('max_speed', 0.3))
        del kwargs['max_speed']

        jids = [j for j, val in enumerate(vals) if val is not None]
        value = [v for v in vals if v is not None]
        assert len(jids) == len(value), \
            'Input number of position values must match the number of joints'

        command = {self._joints[i]: value[i] for i in range(len(value))}

        # blocking call
        self._limb.move_to_joint_positions(command, **kwargs)

    @joint_velocities.setter
    def joint_velocities(self, value):
        """
        Set joint velocities according to given values list. Note
        that the length of the list must match that of the
        joint indices, and if one can skip certain joints by
        setting those values to None.
        :param value: A list of length DOF.
        :return: None
        """
        if self._safe:
            assert len(value) == len(self._joints)
            self._command_msg.mode = JointCommand.VELOCITY_MODE
            self._command_msg.velocity = list(value)
            self._command_msg.header.stamp = rospy.Time.now()
            self._safenet.publish(self._command_msg)
        else:
            jids = [j for j, val in enumerate(value) if val is not None]
            value = [v for v in value if v is not None]
            assert len(jids) == len(value), \
                'Input number of position values must match the number of joints'

            command = {self._joints[i]: value[i] for i in range(len(value))}
            self._limb.set_joint_velocities(command)

    def _msg_wrapper(self, ctype): 

        if len(self.cmd) == len(self._joints):
            if ctype == 'velocity':
                self._command_msg.mode = JointCommand.VELOCITY_MODE
                self._command_msg.velocity = self.cmd

            elif ctype == 'torque':
                self._command_msg.mode = JointCommand.TORQUE_MODE
                self._command_msg.effort = self.cmd
            else:
                raise NotImplemented

            self._command_msg.header.stamp = rospy.Time.now()
            self._safenet.publish(self._command_msg)

    @joint_torques.setter
    def joint_torques(self, value):
        """
        Set joint torques according to given values list. Note
        that the length of the list must match that of the
        joint indices, and if one can skip certain joints by
        setting those values to None.
        :param value: A list of length DOF.
        :return: None
        """
        if self._safe:
            assert len(value) == len(self._joints)
            self._command_msg.mode = JointCommand.TORQUE_MODE
            self._command_msg.effort = list(value)
            self._command_msg.header.stamp = rospy.Time.now()
            self._safenet.publish(self._command_msg)
        else:
            jids = [j for j, val in enumerate(value) if val is not None]
            value = [v for v in value if v is not None]
            assert len(jids) == len(value), \
                'Input number of position values must match the number of joints'

            command = {self._joints[i]: value[i] for i in range(len(value))}
            self._limb.set_joint_torques(command)

    def spin(self, rate=100, ctype='velocity'):

        self._commanders[ctype] = Commander(
            rate, self._msg_wrapper, 
            # message wrapper arguments
            ctype)

        self._commanders[ctype].start()

    def set_timeout(self, time):
        """
        Set the timeout in seconds for the joint controller
        :param timeout: float timeout in seconds
        """
        self._limb.set_command_timeout(time)

    def show_image(self, image_path, rate=1.0):
        """
        Display given image on sawyer head display
        :param image_path: absolute path string of the image
        :param rate: refresh rate
        :return: None
        """
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
    def light(self, name_on):
        """
        Set the status of given light
        :param name: string name of the light
        :param on: boolean True for on, False for off
        :return: True if light state is set, False if not
        """
        name_on = (name, on)
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

    # TODO: Use sawyer internal IK for now
    @tool_pose.setter
    def tool_pose(self, pose):
        """
        Set the end effector pose
        :param pose: (pos, orn) tuple vec3 float cartesian in robot base frame,
        and vec4 float quaternion in robot base frame.
        """
        if self.motion_planning:
            gripper_pose = Pose()

            gripper_pose.position.x = pose[0][0]
            gripper_pose.position.y = pose[0][1]
            gripper_pose.position.z = pose[0][2]

            # TODO: need a transformation for pose
            gripper_pose.orientation.w = pose[1][3]
            gripper_pose.orientation.x = pose[1][0]
            gripper_pose.orientation.y = pose[1][1]
            gripper_pose.orientation.z = pose[1][2]
            
            self._group.set_pose_target(gripper_pose)
            plan = self._group.plan()

            if len(plan.joint_trajectory.points) == 0:
                print('No viable plan to reach target pose.')
                return False
            else:
                self._group.execute(plan)
        else:
            ns = "ExternalTools/right/PositionKinematicsNode/IKService"
            iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
            ikreq = SolvePositionIKRequest()
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            poses = {
                'right': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(*pose[0]),
                        # Initialized as x y z w
                        orientation=Quaternion(*pose[1]),
                    ),
                ),
            }
            # Add desired pose for inverse kinematics
            ikreq.pose_stamp.append(poses["right"])
            # Request inverse kinematics from base to "right_hand" link
            ikreq.tip_names.append('right_hand')

            try:
                rospy.wait_for_service(ns, 5.0)
                resp = iksvc(ikreq) # get IK response
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % (e,))
                return False
            if resp.result_type[0] > 0:
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                self._limb.move_to_joint_positions(limb_joints)
                rospy.loginfo("Move to position succeeded")
            else:
                rospy.logerr("IK response is not valid")
                return False

    def set_max_speed(self, factor):

        self._limb.set_joint_position_speed(factor)
        if self.motion_planning:
            self._group.set_max_velocity_scaling_factor(factor)

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
        for commander in self._commanders.values():
            if commander is not None:
                commander.pause()

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
        for commander in self._commanders.values():
            if commander is not None:
                commander.resume()

    def reset(self, reboot=False):
        """
        Reset the robot and move to rest pose
        :return: None
        """
        self.cmd = []
        if reboot:
            if self._robot_enable.state().error:
                self._robot_enable.reset()
            if self._has_gripper:
                self._gripper.reboot()
                self._gripper.calibrate()
        self._limb.move_to_neutral()
        if self._has_gripper:
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

        for commander in self._commanders.values():
            if commander is not None:
                commander.cancel()

    @staticmethod
    def shutdown():
        """
        Safely shut down the robot controller
        :return: None
        """
        rospy.signal_shutdown('Safely shut down Sawyer.')

    def move_to(self, x, y, z, orn=(0, 1, 0, 0)):
        """
        Move to given position.
        :param x: x coordinate position relative to robot base
        :param y: y coordinate position relative to robot base
        :param z: z coordinate position relative to robot base
        :return: None
        """
        self.tool_pose = ((x, y, z), orn)

    def move_to_with_grasp(self, x, y, z, hover, dive, orn=(0, 1, 0, 0)):
        """
        Move to given position and grasp
        :param x: refer to <move_to::x>
        :param y: refer to <move_to::y>
        :param z: refer to <move_to::z>
        :param hover: the distance above object before grasping, in meters
        :param dive: the distance before gripper slows down
        for a grasp, in meters
        :return: None
        """
        self.grasp(1)
        self.set_max_speed(0.15)
        self.move_to(x, y, z + hover, orn)
        self._safe = True
        time.sleep(0.7)
        self.move_to(x, y, z + dive, orn)
        self.set_max_speed(0.05)
        self.move_to(x, y, z - FINGER_OFFSET, orn)
        time.sleep(0.8)
        self.grasp(0)

    def move_to_with_lift(self,
                          x, y, z,
                          hover=0.4,
                          dive=0.05,
                          drop=True,
                          drop_height=0.3,
                          orn=(0, 0, 0, 1)):
        """
        Move to given position, grasp the object,
        and lift up the object.
        :param x: refer to <move_to::x>
        :param y: refer to <move_to::y>
        :param z: refer to <move_to::z>
        :param hover: refer to <move_to_with_grasp::hover>
        :param dive: refer to <move_to_with_grasp::dive>
        :param drop: boolean whether drop the object after lift
        :param drop_height: the height when releasing grasped object
        :return: None
        """
        self._safe = False
        self.move_to_with_grasp(x, y, z, hover, dive)
        time.sleep(.75)
        self.set_max_speed(0.1)
        self._safe = True
        self.move_to(x, y, z + hover)
        time.sleep(0.2)

        self.move_to(x, y, z + drop_height)
        time.sleep(.8)

        if drop:
            self.grasp(1)
            time.sleep(.5)
