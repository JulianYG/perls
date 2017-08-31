from __future__ import print_function
import pybullet as p
from perls.src.lib.utils.io_util import parse_log as plog
from perls.src.lib.utils.math_util import get_relative_pose, get_absolute_pose
from glob import glob
import numpy as np
import matplotlib.pyplot as plt
from IPython import embed
import gym


class Postprocess(object):
    def __init__(self, robot_base_pose, objects_fname="sim_/log/trajectory/body_info.txt"):

        # this one is for computing relative poses
        self.robot_base_pose = robot_base_pose

        # this one is for loading the robot for FK
        robot_base_pose = (np.array([-0.15, -0.2, 0.9]), np.array([0, 0, 0, 1]))

        ### TODO: why should these be different... FUCK bullet

        p.connect(p.DIRECT)
        p.setRealTimeSimulation(0)
        self.robot_file = p.loadURDF("/Users/ajaymandlekar/Desktop/Dropbox/Stanford/ccr/bullet3/data/sawyer_robot/sawyer_description/urdf/sawyer_arm.urdf",
                          robot_base_pose[0], robot_base_pose[1], useFixedBase=True)
        p.resetBasePositionAndOrientation(self.robot_file, robot_base_pose[0], robot_base_pose[1])

        ### TODO: why does the base pose change here and why is the above line necessary???
        # self.robot_base_pose = (p.getLinkState(self.robot_file, 0)[4], p.getLinkState(self.robot_file, 0)[5])

        # get mapping between entity ids and entity names
        # self.object_map = {}
        # objects_fptr = open(objects_fname, "r")
        # lines = objects_fptr.readlines()
        # objects_fptr.close()
        # for line in lines:
        #     obj_id, obj_name = line.strip().split(",")
        #     self.object_map[int(obj_id)] = obj_name

        # self.object_ids = [int(x) for x in self.object_map.keys()]
        # self.object_names = self.object_map.values()

        # hardcoded mapping between entity ids and entity names
        self.object_map = {4: "cube_0", 1: "titan_0"}
        self.object_ids = [int(x) for x in self.object_map.keys()]
        self.object_names = self.object_map.values()

        # list to map column index to name
        self.col_names = [u'stepCount', u'timeStamp', u'objectId', u'posX', u'posY', u'posZ',
                          u'oriX', u'oriY', u'oriZ', u'oriW', u'velX', u'velY', u'velZ',
                          u'omegaX', u'omegaY', u'omegaZ', u'qNum',
                          u'q0', u'q1', u'q2', u'q3', u'q4', u'q5', u'q6',
                          u'v0', u'v1', u'v2', u'v3', u'v4', u'v5', u'v6',
                          u'u0', u'u1', u'u2', u'u3', u'u4', u'u5', u'u6']

        # dictionary to map column name to index
        self.col_names_dict = {}
        for i, name in enumerate(self.col_names):
            self.col_names_dict[name] = i

    def parse_log(self, fname, output_file, verbose=True, objects=None, cols=None):
        """

        This function parses a log saved by PyBullet, but it filters the
        saved objects on the provided keys and the collected data on cols.
        Both keys and cols should be lists of strings.

        Example usage below:

        Parse a log for data concerning the robot arm and gripper.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"])

        Same as above, but filter out only the joint positions.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"],
                            cols=['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8', 'q9', 'q10', 'q11'])

        """

        log = np.array(plog(fname, verbose=verbose))

        ### Important: Toss the first 2500 rows.
        log = log[2500:]

        col_inds = sorted(self.col_names_dict.values())
        if cols is not None:
            # make sure desired columns are valid
            assert (set(cols) <= set(self.col_names))

            # get column inds to filter out
            col_inds = map(self.col_names_dict.get, cols)

        # get object ids to filter out
        if objects is None:
            filter_object_ids = self.object_ids
        else:
            filter_object_ids = []
            for obj in objects:
                for obj_id in self.object_map:
                    if obj in self.object_map[obj_id]:
                        filter_object_ids.append(obj_id)

        ind = self.col_names_dict['objectId']

        def filter_row(row):
            if row[ind] in filter_object_ids:
                return row[col_inds]
            else:
                return None

        filtered = map(filter_row, log)
        return np.array([x for x in filtered if x is not None])

    def fk(self, joint_pos):
        """
        WARNING: returns eef pose in world frame
        """

        for i in range(7):
            p.resetJointState(self.robot_file, i, joint_pos[i])

        x = p.getLinkState(self.robot_file, 6)
        return x[4], x[5]

    def parse_demonstration(self, fname):
        """
        Parse a bullet bin file into states and actions.
        """
        robot_log = self.parse_log(fname, None, verbose=False, objects=["titan_0"],
                                   cols=['stepCount', 'timeStamp', 'qNum',
                                         'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                                         'v0', 'v1', 'v2', 'v3', 'v4', 'v5', 'v6',
                                         'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6'])

        cube_log = self.parse_log(fname, None, verbose=False, objects=["cube_0"],
                                  cols=['stepCount', 'timeStamp', 'qNum', 'posX', 'posY', 'posZ', 'oriX', 'oriY', 'oriZ',
                                        'oriW'])

        # Time differences.
        time_diffs = robot_log[1:, 1] - robot_log[:-1, 1]

        # print("Time diff: {}".format(time_diffs[0]))


        # First subsample one of every 24 points to get control roughly at 10 Hz
        num_elems = min(robot_log.shape[0], cube_log.shape[0])
        filt_robot_log = []
        filt_cube_log = []
        for i in range(num_elems):
            if i % 24 == 0:
            #if i % 1 == 0:
                filt_robot_log.append(robot_log[i])
                filt_cube_log.append(cube_log[i])

        robot_log = np.array(filt_robot_log)
        cube_log = np.array(filt_cube_log)
        print("Number of elems before subsampling: {}".format(num_elems))
        print("Number of elems after subsampling: {}".format(robot_log.shape[0]))

        num_elems = robot_log.shape[0]
        num_elems1 = cube_log.shape[0]
        assert(num_elems == num_elems1)

        ### TODO: add in orientation too...

        states = []
        actions = []
        timestamps = []

        num_filtered = 0
        prev_joint_pos = robot_log[0, 3:10]
        prev_cube_pose = (cube_log[0, 3:6], cube_log[0, 6:10])
        prev_cube_pose_pos, prev_cube_pose_orn = get_relative_pose(prev_cube_pose, self.robot_base_pose)
        prev_eef_pose = self.fk(prev_joint_pos)
        prev_eef_pose_pos, prev_eef_pose_orn = get_relative_pose(prev_eef_pose, self.robot_base_pose)
        print("Initial joint angles: {}".format(prev_joint_pos))
        print("Initial eef pose in world frame: {}".format(prev_eef_pose))
        print("Initial eef pose in robot frame: {}".format((prev_eef_pose_pos, prev_eef_pose_orn)))
        print("Initial eef position: {}".format(prev_eef_pose_pos))
        cube_initial_z = prev_cube_pose_pos[-1]
        print("Initial cube z-location: {}".format(cube_initial_z))

        print("Using robot pose: {}".format(self.robot_base_pose))


        for i in range(1, num_elems):
            timestamp_elem = robot_log[i, 1]
            joint_pos_elem = robot_log[i, 3:10]
            joint_vel_elem = robot_log[i, 10:17]
            joint_torq_elem = robot_log[i, 17:24]
            cube_pose_elem = (cube_log[i, 3:6], cube_log[i, 6:10])

            # convert from world frame to robot frame
            cube_pose_pos_elem, cube_pose_orn_elem = get_relative_pose(cube_pose_elem, self.robot_base_pose)
            eef_pose_pos_elem, eef_pose_orn_elem = get_relative_pose(self.fk(joint_pos_elem), self.robot_base_pose)

            # filter on eef positions being similar (user didn't move) and cube falling
            if (i != 1) and (np.all(np.absolute(np.array(eef_pose_pos_elem) - np.array(prev_eef_pose_pos)) < 1e-5) \
                             or (prev_cube_pose_pos[-1] < cube_initial_z - 0.01)):
                prev_joint_pos = joint_pos_elem
                prev_eef_pose_pos, prev_eef_pose_orn = eef_pose_pos_elem, eef_pose_orn_elem
                prev_cube_pose_pos, prev_cube_pose_orn = cube_pose_pos_elem, cube_pose_orn_elem
                num_filtered += 1
                continue   

            # filter on joint positions being similar (user didn't move) and cube falling
            # note that we always take the first element
            # if (i != 1) and (np.all(np.absolute(np.array(joint_pos_elem) - np.array(prev_joint_pos)) < 1e-5) \
            #                  or (prev_cube_pose_pos[-1] < cube_initial_z - 0.01)):
            #     print(joint_pos_elem)
            #     prev_joint_pos = joint_pos_elem
            #     prev_eef_pose_pos, prev_eef_pose_orn = eef_pose_pos_elem, eef_pose_orn_elem
            #     prev_cube_pose_pos, prev_cube_pose_orn = cube_pose_pos_elem, cube_pose_orn_elem
            #     num_filtered += 1
            #     continue

            ### State and Action definition here ###

            # NOTE: we add the joint angles in state, joint vels in action
            state = np.concatenate([joint_pos_elem, cube_pose_pos_elem, cube_pose_orn_elem])
            action = np.array(joint_vel_elem)

            # NOTE: we add the previous eef orientation here, and previous cube orientation
            # state = np.concatenate([prev_eef_pose_pos, prev_cube_pose_pos, prev_cube_pose_orn])
            # action = np.array(eef_pose_pos_elem) - np.array(prev_eef_pose_pos)


            states.append(state)
            actions.append(action)
            timestamps.append(timestamp_elem)

            # remember last positions for filtering and relative stuff
            prev_joint_pos = joint_pos_elem
            prev_eef_pose_pos, prev_eef_pose_orn = eef_pose_pos_elem, eef_pose_orn_elem
            prev_cube_pose_pos, prev_cube_pose_orn = cube_pose_pos_elem, cube_pose_orn_elem

        print("Number filtered: {} out of {}.".format(num_filtered, num_elems))
        print("Last cube z-position: {}".format(states[-1][5]))
        
        # timestamps = np.array(timestamps)
        # time_diffs = timestamps[1:] - timestamps[:-1]

        # plt.figure()
        # plt.plot(time_diffs)
        # plt.show()

        return np.array(states), np.array(actions)


if __name__ == "__main__":

    env = gym.make('push-vel-v0')
    env.reset()

    # robot_position = env._robot.pos
    # robot_orn = env._robot.orn
    # robot_pose = (robot_position, robot_orn)
    robot_base_pose = env._robot.pose
    # robot_base_pose = (np.array([-0.15, -0.2, 0.9]), np.array([0, 0, 0, 1]))
    env.close()
    plt.close("all") # dirty haxxx

    # hard-coded robot base pose
    # robot_position = [-0.2, -0.7, 0.9]
    # robot_orn = [0.0, 0.0, 0.0, 1.0]
    # robot_pose = (robot_position, robot_orn)

    #fname = "success/2017-08-27-22-08-35.bin"
    #fname = "success/2017-08-27-22-11-11.bin"
    #fname = "success/2017-08-27-22-12-14.bin"

    pp = Postprocess(robot_base_pose)

    all_states = list()
    all_actions = list()

    ### Test FK here.
    # joint_angles = np.array([-0.406434179930325, -0.5088564232765723, -0.01815209772386096, 2.1507001664115046, -0.22093926951517517, -0.07271383292633793, 3.122028481588129])
    # tmp = pp.fk(joint_angles)
    # print("EEF pose after FK: {}".format(tmp))
    # tmp = (np.array([ 0.252, -0.208,  0.84 ]), np.array([0., 1., 0., 0.]))
    # print("EEF pose hardcoded: {}".format(tmp))



    for fname in glob("success/*.bin"):
    #for fname in glob("2017-08-29-11-48-34.bin"):
    #for fname in glob("success/2017-08-27-22-08-35.bin"):
        states, actions = pp.parse_demonstration(fname)
        all_states.append(states)
        all_actions.append(actions)
    all_states = np.concatenate(all_states, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    print(all_states.shape)
    print(all_actions.shape)
    np.savez("demo_joint_vel.npz", states=all_states, actions=all_actions)












