from __future__ import print_function 
#import pybullet as p
#from sim_.simulation.utils.io import parse_log as plog
from perls.src.lib.utils.io_util import parse_log as plog
from perls.src.lib.utils.math_util import get_relative_pose, get_absolute_pose
from glob import glob
import numpy as np
import matplotlib.pyplot as plt
from IPython import embed

# hard-coded robot base pose
robot_position = [-0.6756339993327856, 0.010968999937176704, 1.1236299961805343]
robot_orn = [0.0, 0.0, 0.0, 1.0]
robot_pose = (robot_position, robot_orn)


class Postprocess(object):
    def __init__(self, objects_fname="sim_/log/trajectory/body_info.txt"):

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

        # hardcoded mapping between entity ids and entity ames
        self.object_map = {4 : "cube_0", 1 : "titan_0"}
        self.object_ids = [int(x) for x in self.object_map.keys()]
        self.object_names = self.object_map.values()

        # list to map column index to name
        self.col_names = [u'stepCount', u'timeStamp', u'objectId', u'posX', u'posY', u'posZ', 
                          u'oriX', u'oriY', u'oriZ', u'oriW', u'velX', u'velY', u'velZ', 
                          u'omegaX', u'omegaY', u'omegaZ', u'qNum', 
                          u'q0', u'q1', u'q2', u'q3', u'q4', u'q5', u'q6', u'q7', u'q8', u'q9', u'q10', u'q11',
                          u'v0', u'v1', u'v2', u'v3', u'v4', u'v5', u'v6', u'v7', u'v8', u'v9', u'v10', u'v11',
                          u'u0', u'u1', u'u2', u'u3', u'u4', u'u5', u'u6', u'u7', u'u8', u'u9', u'u10', u'u11']

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

        col_inds = sorted(self.col_names_dict.values())
        if cols is not None:
            # make sure desired columns are valid
            assert(set(cols) <= set(self.col_names))

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


    def parse_demonstration(self, fname):
        """
        Parse a bullet bin file into states and actions.
        """
        robot_log = self.parse_log(fname, None, verbose=False, objects=["titan_0"], 
                                   cols=['stepCount', 'timeStamp', 'qNum', 
                                         'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                                         'v0', 'v1', 'v2', 'v3', 'v4', 'v5', 'v6',
                                         'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6'])

        cube_log = pp.parse_log(fname, None, verbose=False, objects=["titan_0"], 
                                cols=['stepCount', 'timeStamp', 'qNum', 'posX', 'posY', 'posZ', 'oriX', 'oriY', 'oriZ', 'oriW'])


        # Time differences. 
        # time_diffs = robot_log[1:, 1] - robot_log[:-1, 1]

        # plt.figure()
        # plt.plot(time_diffs)
        # plt.show()

        #embed()

        num_elems = robot_log.shape[0]
        states = []
        actions = []

        num_filtered = 0
        prev_joint_pos = robot_log[0, 3:10]

        for i in range(1, num_elems):
            timestamp_elem = robot_log[i, 1]
            joint_pos_elem = robot_log[i, 3:10]

            # TODO: think about more natural filtering mechanism here?

            # filter on joint positions being similar (user didn't move)
            if np.all(np.absolute(np.array(joint_pos_elem) - np.array(prev_joint_pos)) < 1e-4):
                prev_joint_pos = joint_pos_elem
                num_filtered += 1
                continue

            joint_vel_elem = robot_log[i, 10:17]
            joint_torq_elem = robot_log[i, 17:24]
            cube_pose_elem = (cube_log[i, 3:6], cube_log[i, 6:10])
            #cube_pose_pos_elem, cube_pose_orn_elem = cube_pose_elem

            # convert from world frame to robot frame
            cube_pose_pos_elem, cube_pose_orn_elem = get_relative_pose(cube_pose_elem, robot_pose)


            state = np.concatenate([joint_pos_elem, joint_vel_elem, cube_pose_pos_elem, cube_pose_orn_elem])
            action = np.array(joint_torq_elem)
            states.append(state)
            actions.append(action)

            # remember last joint position for filtering
            prev_joint_pos = joint_pos_elem

        print("Number filtered: {} out of {}.".format(num_filtered, num_elems))
        return np.array(states), np.array(actions)


if __name__ == "__main__":

    fname = "test.bin"
    pp = Postprocess()
    pp.parse_demonstration(fname)











