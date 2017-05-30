# !/usr/bin/env python
import intera_interface.cfg.SawyerVelocityJointTrajectoryActionServerConfig as cfg
import math
import numpy as np

import rospy
from dynamic_reconfigure.server import Server
from intera_interface import CHECK_VERSION
import intera_interface
import intera_control

# TODO: the following applies to the online_trajectory.py file
# TODO: restructure code to:
# 1. remove goal, put our own data structure
# 2. recover self.dyn_config, maybe hardcode them / pickle it?
# 3. put interpolation between subsequent points, see performance (to mimic online setting)
# 4. remove Server class too

class RobotController(object):
    """
    This class is for controlling the Robot with a sequence of joint states and timestamps.
    It is meant for teleoperation.
    """
    def __init__(self, limb_name="right", rate=100, command_queue=None):
        """
        Note: method assumes rospy.init_node() has already been called.

        :param limb: Name of the limb. There should be no need to change this.
        :param rate: The control rate, in Hz.
        :param command_queue: Should be an instance of the Python Queue class.
        """

        # TODO: should we have this call rospy.init_node()

        # the name of the limb
        self.limb_name = limb_name

        # Limb object from Intera SDK
        self.limb = intera_interface.Limb(limb_name)
        self.cuff = intera_interface.Cuff(limb=limb_name)

        self._action_name = rospy.get_name()
        self._enable = intera_interface.RobotEnable()

        # control rate in Hz
        self.control_rate = rate

        # TODO: do we need the gripper?
        # get the gripper
        # self.gripper_name = '_'.join([limb, 'gripper'])
        # self.gripper = intera_interface.Gripper(limb)

        # ordered list of joint names
        self.joint_names = self.limb.joint_names()
        self.num_joints = len(self.joint_names)
        print("Joint names in order are : {}".format(self.joint_names))

        # TODO: put max joint velocity here, integrate into _min_jerk_coeffs to lower bound duration.
        self.max_joint_velocity = None

        # TODO: keep track of received joint positions in a queue?

        # keep track of last joint positions received in an array ordered by self.joint_names
        self.last_pos = None

        # TODO: look at where these are stored, should we change these settings?
        ### Configuration settings. ###
        self._dyn = Server(cfg, lambda config, level: config)
        path_error = self._dyn.config[jnt + '_trajectory']
        self._path_thresh = dict(zip(self.joint_names, [path_error] * self.num_joints))

        # Create PID controllers per joint.
        self._pid = dict()
        for jnt in self.joint_names:
            self._pid[jnt] = intera_control.PID()
            self._pid[jnt].set_kp(self._dyn.config[jnt + '_kp'])
            self._pid[jnt].set_ki(self._dyn.config[jnt + '_ki'])
            self._pid[jnt].set_kd(self._dyn.config[jnt + '_kd'])
            self._pid[jnt].initialize()
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self.control_rate)


        # reset to base position
        self.reset()

        # TODO: make sure command queue allows us to do asynch polling
        self.command_queue = command_queue
        assert (self.command_queue is not None)

    def reset(self):
        """
        This function just resets the arm to the rest position.
        """
        self.limb.move_to_neutral()
        self.last_pos = np.array(self._get_current_position())

    def _minjerk_computation(self, points_array, duration_array=None):
        """
        NOTE: This function was taken from the Intera SDK. We must include the
        copyright notice from minjerk.py, if we wish to use this.

        Compute the min-jerk coefficients for a given set for user-supplied control pts

        params:
           points_array: array of user-supplied control points
               numpy.array of size N by k
               N is the number of control points
               k is the number of dimensions for each point
           duration_array: array of user-supplied control duration of ech segment
               numpy.array of size N-1
               N is the number of control points

        returns:
          m_coeffs:  k-dimensional array of N-1 x (6 coefficients + 1 duration of each segment)
               numpy.array of size N-1 by (6+1) by k
        """
        (rows, k) = np.shape(points_array)
        N = rows - 1  # N minus 1 because points array includes x_0
        m_coeffs = np.zeros(shape=(k, N, 7))
        x = points_array[0]
        v = np.zeros(k)
        a = np.zeros(k)
        if duration_array == None:
            duration_array = np.array([1.0] * N)
        assert len(duration_array) == N, \
            "Invalid number of intervals chosen (must be equal to N+1={})".format(N)
        for i in range(0, N):
            gx = points_array[i + 1];
            t = duration_array[i]
            if i == N - 1:
                gv = np.zeros(k)
            else:
                t0 = t
                t1 = duration_array[i + 1]
                d0 = points_array[i + 1] - points_array[i]
                d1 = points_array[i + 2] - points_array[i + 1]
                v0 = d0 / t0
                v1 = d1 / t1
                gv = np.where(np.multiply(v0, v1) >= 1e-10, 0.5 * (v0 + v1), np.zeros(k))  # 0 + eps
            ga = np.zeros(k)

            A = (gx - (x + v * t + (a / 2.0) * t * t)) / (t * t * t);
            B = (gv - (v + a * t)) / (t * t);
            C = (ga - a) / t;

            a0 = x;
            a1 = v;
            a2 = a / 2.0;
            a3 = 10 * A - 4 * B + 0.5 * C;
            a4 = (-15 * A + 7 * B - C) / t;
            a5 = (6 * A - 3 * B + 0.5 * C) / (t * t);

            x = gx
            v = gv

            m_coeffs[:, i, 0] = a0
            m_coeffs[:, i, 1] = a1
            m_coeffs[:, i, 2] = a2
            m_coeffs[:, i, 3] = a3
            m_coeffs[:, i, 4] = a4
            m_coeffs[:, i, 5] = a5
            m_coeffs[:, i, 6] = t
        return m_coeffs

    def _minjerk_coeffs(self, p1, p2, duration):
        """
        This function does minjerk interpolation between two sets of joint angles.
        It returns the coefficients for the smooth polynomial interpolation functions.

        :param p1: An ordered array of joint angles at the beginning of the time interval.
        :param p2: An ordered array of joint angles at the end of the time interval.
        :param duration: Time it should take to move from p1 to p2.

        :return: An array of coefficients of shape (num_joints, 7)
        """

        return self._minjerk_computation(np.array([p1, p2]), [duration])[:, 0, :]

    def _minjerk_points(self, coeffs, num_pts):
        """
        Uses the minjerk @coeffs to return @num_pts interpolated points between two control points,
        including the start point.

        :param coeffs: minjerk coefficients of shape num_joints by 7
        :param num_pts: number of desired interpolated points between the
                        two control points, including the start point

        :return: array of interpolated points of shape (num_joints, num_pts)
        """

        num_joints, _ = coeffs.shape

        # absolute time between start and end points
        tm = coeffs[:, 6]

        # make sure all the segment times are equal
        assert(np.array_equal(tm, tm[0] * np.ones(num_joints)))

        # generate evenly spaced points from [0, 1)
        t_pts = np.linspace(0, 1, num_pts, endpoint=False)

        # convert percentage times to absolute times
        t = (t_pts * tm[0]).reshape((-1, 1))

        # use a clever matrix multiplication to return all interpolated points at once
        t_mat = np.concatenate([np.ones((num_pts, 1)), t, np.power(t, 2), np.power(t, 3), np.power(t, 4), np.power(t, 5)], axis=1)
        A = coeffs[:, :6]
        return A.dot(t_mat.T)

    def interpolate(self, new_pos, duration, num_pts):
        """
        This function does minjerk interpolation between the new position and the last position, returning
        num_pts number of points.

        :param new_pos: An ordered array of joint angles to move to.
        :param duration: The amount of time in seconds that it should take to move there from the last position.
        :param num_pts: The number of desired interpolated points, including the last position.
        :return: Array of interpolated points of shape (num_joints, num_pts)
        """
        coeffs = self._minjerk_coeffs(self.last_pos, new_pos, duration)
        return self._minjerk_points(coeffs, num_pts)

    def minjerk_t(self, coeffs, t):
        """
        This function evaluates the minjerk interpolated path at the passed time t.

        :param coeffs: minjerk coefficients of shape num_joints by 7
        :param t: ratio of time passed, in [0, 1]

        :return: a set of joint positions along the minjerk trajectory
        """
        t_vec = np.array([1.0, t, t ** 2, t ** 3, t ** 4, t ** 5]).reshape((-1, 1))
        A = coeffs[:, :6]
        return A.dot(t_vec)

    def robot_is_enabled(self):
        """
        Returns whether robot is enabled.
        """
        return self._enable.state().enabled

    def _get_current_position(self):
        """
        Returns current robot joint angles as a numpy array.
        """
        return np.array([self.limb.joint_angle(joint) for joint in self.joint_names])

    def _get_current_velocities(self):
        """
        Returns current robot joint velocities as a numpy array.
        """
        return np.array([self.limb.joint_velocity(joint) for joint in self.joint_names])

    def _get_current_error(self, set_point):
        """
        Returns an array of differences between the desired joint positions and current joint positions.

        :param set_point: the joint positions that are desired
        :return: the current error in the joint positions
        """
        current = self._get_current_position()
        error = set_point - current
        return error

    def control_loop(self):
        """
        This function encapsulates the main purpose of this class, which is to receive commanded joint
        positions via the command queue, and control the robot by interpolating between the commanded
        joint positions and using PID controllers per joint to set joint velocities accordingly.
        """

        # helps us keep the specified control rate
        control_rate = rospy.Rate(self.control_rate)

        while True:

            # if no new points, maintain current position by writing 0 to all joint velocities
            if self.command_queue.empty():
                self.limb.set_joint_velocities(dict(zip(self.joint_names, [0.0] * self.num_joints)))
                control_rate.sleep()
                continue

            # dequeue new position
            new_pos, duration = self.command_queue.get()

            # do minjerk interpolation to determine the new trajectory
            current_coeffs = self._minjerk_coeffs(self.last_pos, new_pos, duration)

            # loop for the duration of the motion between the control points
            start_time = rospy.get_time()
            now_from_start = rospy.get_time() - start_time
            while (now_from_start < duration):

                # get interpolated point using ratio of time expired in time slice
                now_from_start = rospy.get_time() - start_time
                t = np.clip(now_from_start / duration, 0.0, 1.0)
                inter_pos = self.minjerk_t(current_coeffs, t)

                # some error checking
                if self.limb.has_collided() or not self.robot_is_enabled() or self.cuff.cuff_button():
                    rospy.logerr("{0}: Robot arm in Error state. Stopping execution.".format(
                        self._action_name))
                    self.limb.exit_control_mode()
                    return False

                # use PID controllers to control the arm via joint velocities
                velocities = dict()
                deltas = self._get_current_error(inter_pos)
                for jnt, delta in zip(self.joint_names, deltas):
                    if math.fabs(delta) >= self._path_thresh[jnt] >= 0.0:
                        rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                                     (self._action_name, jnt, str(delta),))
                        self.limb.exit_control_mode()
                        return False
                    velocities[jnt] = self._pid[jnt].compute_output(delta)
                self.limb.set_joint_velocities(velocities)

                # enforce the control rate
                control_rate.sleep()

            # update the last point
            self.last_pos = new_pos






















