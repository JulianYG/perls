import logging 

import pybullet as p

from .math_util import get_relative_pose, vec
from .io_util import parse_log, parse_config, PerlsLogger
from ..control import Controller
import numpy as np
# import cv2
# from PIL import Image
import matplotlib.pyplot as plt
import time
import cv2

logging.setLoggerClass(PerlsLogger)


class Postprocess:

    def __init__(self, modality, conf_path, verbose=False, dim='low', use_display=False):

        conf = parse_config(conf_path)[0]
        _, world, display, _ = Controller.load_config(conf, None)
        self.verbose = verbose
        self.modality = modality
        self.state_dim = dim
        self.use_display = use_display

        display.run()
        display.set_render_view(
            dict(
                dim=(150, 150),
                flen=2.,
                # Have to use exact numbers for aligned
                # Top down view in GUI and non-GUI modes...
                yaw=90.0001,
                pitch=-75,
                focus=world.body['table_0'].pos
            )
        )
        self.display = display
        world.boot('cmd')
        world.reset()

        self.robot = world.body['titan_0']
        self.table = world.body['table_0']
        self.cube = world.body['cube_0']

        self.object_map = {4: "cube_0", 1: "titan_0", 0: 'bax_0'}
        self.object_ids = [int(x) for x in self.object_map.keys()]
        self.object_names = self.object_map.values()
        self.col_names = [u'stepCount', u'timeStamp', u'objectId', u'posX', u'posY', u'posZ',
                          u'oriX', u'oriY', u'oriZ', u'oriW', u'velX', u'velY', u'velZ',
                          u'omegaX', u'omegaY', u'omegaZ', u'qNum',
                          u'q0', u'q1', u'q2', u'q3', u'q4', u'q5', u'q6',
                          u'u0', u'u1', u'u2', u'u3', u'u4', u'u5', u'u6',
                          u't0', u't1', u't2', u't3', u't4', u't5', u't6']

        # dictionary to map column name to index
        self.col_names_dict = {}
        for i, name in enumerate(self.col_names):
            self.col_names_dict[name] = i

        display.show()

    @property
    def name(self):

        if self.state_dim == 'low':
            return self.modality
        else:
            return self.state_dim

    def parse(self, fname, objects=None, cols=None):
        """
        This function parses a log saved by PyBullet, but it filters the
        saved objects on the provided keys and the collected data on cols.
        Both keys and cols should be lists of strings.

        Example usage below:
        Parse a log for data concerning the robot arm and gripper.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"])

        Same as above, but filter out only the joint positions.

            log = parse_log("data.bin", "out.txt", objects=["sawyer", "gripper"],
                            cols=['q0', 'q1', 'q2', 'q3', 'q4', 'q5',
                                  'q6', 'q7', 'q8', 'q9', 'q10', 'q11'])
        """
        log = np.array(parse_log(fname, verbose=self.verbose))
        col_inds = sorted(self.col_names_dict.values())

        if cols is not None:
            # get column inds to filter out
            col_inds = list(map(self.col_names_dict.get, cols))

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

        filtered = list(map(filter_row, log))
        return np.array([x for x in filtered if x is not None])

    def parse_demonstration(self, fname, goal_pos):
        """
        Parse a bullet bin file into states and actions.
        """

        if self.state_dim != 'low':
            disp_im = None

        robot_log = self.parse(
            fname, objects=["titan_0"],
            cols=['stepCount', 'timeStamp', 'qNum',
                  'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                  'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6',
                  't0', 't1', 't2', 't3', 't4', 't5', 't6'])

        cube_log = self.parse(
            fname, objects=["cube_0"],
            cols=['stepCount', 'timeStamp', 'qNum',
                  'posX', 'posY', 'posZ',
                  'oriX', 'oriY', 'oriZ', 'oriW'])

        gripper_log = self.parse(
            fname, objects=["bax_0"],
            cols=['stepCount', 'timeStamp', 'qNum',
                  'posX', 'posY', 'posZ',
                  'oriX', 'oriY', 'oriZ', 'oriW',
                  'q0', 'q1', 'q2', 'q3', 'q4']
        )

        # # Time differences.
        # time_diffs = robot_log[1:, 1] - robot_log[:-1, 1]

        # First subsample one of every 24 points to get control roughly at 10 Hz
        num_elems = min(robot_log.shape[0], cube_log.shape[0], gripper_log.shape[0])
        filt_robot_log = []
        filt_cube_log = []
        filt_gripper_log = []

        for i in range(num_elems):
            if i % 24 == 0:
                filt_robot_log.append(robot_log[i])
                filt_cube_log.append(cube_log[i])
                filt_gripper_log.append(gripper_log[i])

        cube_pos = self.cube.pos

        robot_log = np.array(filt_robot_log)
        cube_log = np.array(filt_cube_log)
        gripper_log = np.array(filt_gripper_log)

        logging.info("Number of elems before subsampling: {}".format(num_elems))
        logging.info("Number of elems after subsampling: {}".format(robot_log.shape[0]))

        num_elems = robot_log.shape[0]
        num_elems1 = cube_log.shape[0]
        assert (num_elems == num_elems1)

        ### TODO: add in orientation too...

        imgs = []
        states = []
        actions = []
        # timestamps = []

        num_filtered = 0
        prev_joint_pos = robot_log[0, 3:10]
        prev_joint_vel = robot_log[0, 10:17]
        prev_cube_pose = (cube_log[0, 3:6], cube_log[0, 6:10])


        # goal_pos, _ = get_relative_pose(
        #     (goal_pos, self.table.orn), self.robot.pose
        # )

        prev_cube_pose_pos, prev_cube_pose_orn = \
            get_relative_pose(prev_cube_pose, self.robot.pose)

        prev_eef_pose = self.robot.eef_pose

        prev_eef_pose_pos, prev_eef_pose_orn = \
            get_relative_pose(prev_eef_pose, self.robot.pose)
        logging.info("Initial joint angles: {}".format(prev_joint_pos))
        logging.info("Initial eef pose in world frame: {}".format(prev_eef_pose))
        logging.info("Initial eef pose in robot frame: {}".format((prev_eef_pose_pos, prev_eef_pose_orn)))
        logging.info("Initial eef position: {}".format(prev_eef_pose_pos))
        cube_initial_z = prev_cube_pose_pos[-1]
        logging.info("Initial cube z-location: {}".format(cube_initial_z))

        logging.info("Using robot pose: {}".format(self.robot.pose))

        for i in range(1, num_elems):

            # timestamp_elem = robot_log[i, 1]
            joint_pos_elem = robot_log[i, 3:10]
            joint_vel_elem = robot_log[i, 10:17]
            # joint_torq_elem = robot_log[i, 17:24]

            gripper_pos = gripper_log[i, 3:6]
            gripper_orn = gripper_log[i, 6:10]

            gripper_joints = gripper_log[i, 10:]

            for j in range(7):
                p.resetJointState(1, j, joint_pos_elem[j])

            for k in range(5):
                p.resetJointState(0, k, gripper_joints[k])

            cube_pose_elem = (cube_log[i, 3:6], cube_log[i, 6:10])
            p.resetBasePositionAndOrientation(4, *cube_pose_elem)
            p.resetBasePositionAndOrientation(0, gripper_pos, gripper_orn)

            # convert from world frame to robot frame
            cube_pose_pos_elem, cube_pose_orn_elem = \
                get_relative_pose(cube_pose_elem, self.robot.pose)
            eef_pose_pos_elem, eef_pose_orn_elem = \
                get_relative_pose(self.robot.eef_pose, self.robot.pose)

            # filter on eef positions being similar (user didn't move) and cube falling
            if (i != 1) and (np.all(np.absolute(np.array(eef_pose_pos_elem) - np.array(prev_eef_pose_pos)) < 1e-5) \
                                     or (prev_cube_pose_pos[-1] < cube_initial_z - 0.01)):
                prev_joint_pos = joint_pos_elem
                prev_eef_pose_pos, prev_eef_pose_orn = eef_pose_pos_elem, eef_pose_orn_elem
                prev_cube_pose_pos, prev_cube_pose_orn = cube_pose_pos_elem, cube_pose_orn_elem
                num_filtered += 1
                continue

            ### State and Action definition here ###

            # NOTE: we add the joint angles in state, joint vels in action (one timestep difference)
            if self.modality == 'vel':
                state = np.concatenate([prev_joint_pos, prev_joint_vel,
                                        prev_cube_pose_pos, prev_cube_pose_orn, goal_pos,
                                        # Additional states with prior knowledge
                                        vec(prev_cube_pose_pos) - vec(prev_eef_pose_pos),
                                        vec(goal_pos) - vec(prev_cube_pose_pos)
                                        ])
                action = np.array(joint_vel_elem)

            # NOTE: we add the previous eef orientation here, and previous cube orientation
            elif self.modality == 'pose':
                state = np.concatenate([prev_eef_pose_pos, prev_cube_pose_pos,
                    prev_cube_pose_orn, goal_pos, 
                    # Additional states with prior knowledge
                    vec(prev_cube_pose_pos) - vec(prev_eef_pose_pos),
                    vec(goal_pos) - vec(prev_cube_pose_pos)
                    ])
                action = np.array(eef_pose_pos_elem) - np.array(prev_eef_pose_pos)
            else:
                return

            if self.state_dim == 'low':
                states.append(state)
                
            # RGBD
            else:
                rgbd = self.display.get_camera_image('rgbd')

                ### Transformations (cropping and ressizing) ###
                rgbd = rgbd[:96, 27:123, :] # 96 x 96 cropping
                # rgbd = cv2.resize(rgbd, (64, 64)) # 64 x 64 resizing
                # print(rgbd.shape, np.max(rgbd), np.min(rgbd))
                imgs.append(rgbd)
                states.append(
                    # No additional states with prior knowledge!!!
                    np.concatenate([
                        prev_joint_pos, prev_joint_vel, goal_pos
                    ]))
                if self.use_display:
                    if disp_im is None:
                        disp_im = plt.imshow(rgbd[:, :, :3])
                    else:
                        disp_im.set_data(rgbd[:, :, :3])
                    plt.pause(0.01)
                    plt.draw()

            actions.append(action)
            
            # remember last positions for filtering and relative stuff
            prev_joint_pos = joint_pos_elem
            prev_joint_vel = joint_vel_elem
            prev_eef_pose_pos, prev_eef_pose_orn = eef_pose_pos_elem, eef_pose_orn_elem
            prev_cube_pose_pos, prev_cube_pose_orn = cube_pose_pos_elem, cube_pose_orn_elem

        logging.info("Number filtered: {} out of {}.".format(num_filtered, num_elems))
        logging.info("Goal region center position: {}".format(goal_pos))

        # timestamps = np.array(timestamps)
        # time_diffs = timestamps[1:] - timestamps[:-1]

        # plt.figure()
        # plt.plot(time_diffs)
        # plt.show()
        if self.state_dim == 'low':
            return np.array(states), np.array(actions)
        else:
            return np.array(imgs), np.array(states), np.array(actions)
