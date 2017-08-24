from six.moves import cPickle as pickle
import numpy as np
from glob import glob
from IPython import embed
import perls
import gym
import time

# add perls/src to path
# import sys
# sys.path.append('/Users/ajaymandlekar/Desktop/Dropbox/Stanford/ccr/code/perls/src')

from perls.src.lib.utils.math_util import get_relative_pose, get_absolute_pose


robot_position = [-0.6756339993327856, 0.010968999937176704, 1.1236299961805343]
robot_orn = [0.0, 0.0, 0.0, 1.0]
robot_pose = (robot_position, robot_orn)

# read file using pickle
def readFile(fname):
    f = open(fname, 'rb')
    data = pickle.load(f)
    f.close()
    return data

# read states and actions from pickle file
def getDemonstration(fname):
    data = readFile(fname)
    arm_data = data['titan_0']
    timestamps = arm_data['time']
    joint_pos = arm_data['joint_position']
    joint_vel = arm_data['joint_velocity']
    joint_torq = arm_data['joint_torque']
    eef = arm_data['pose']

    cube_data = data['cube_0']
    cube_pose = cube_data['pose']

    num_elems = len(joint_pos)
    states = []
    actions = []

    num_filtered = 0
    prev_joint_pos = joint_pos[0]

    for i in range(1, num_elems):
        timestamp_elem = timestamps[i]
        joint_pos_elem = joint_pos[i]

        # TODO: think about more natural filtering mechanism here?

        # filter on joint positions being similar (user didn't move)
        # if np.all(np.absolute(np.array(joint_pos_elem) - np.array(prev_joint_pos)) < 1e-4):
        #     prev_joint_pos = joint_pos_elem
        #     num_filtered += 1
        #     continue

        joint_vel_elem = joint_vel[i]
        joint_torq_elem = joint_torq[i]
        cube_pose_elem = cube_pose[i]
        #cube_pose_pos_elem, cube_pose_orn_elem = cube_pose_elem

        # convert from world frame to robot frame
        cube_pose_pos_elem, cube_pose_orn_elem = get_relative_pose(cube_pose_elem, robot_pose)


        state = np.concatenate([joint_pos_elem, joint_vel_elem, cube_pose_pos_elem, cube_pose_orn_elem])
        action = np.array(joint_torq_elem)
        # action = np.array(joint_vel_elem)
        states.append(state)
        actions.append(action)

        # remember last joint position for filtering
        prev_joint_pos = joint_pos_elem

    print("Number filtered: {} out of {}.".format(num_filtered, num_elems))
    return np.array(states), np.array(actions)

if __name__ == "__main__":
    demons = glob("../Desktop/*.bin")
    fname = demons[0]
    states, actions = getDemonstration(fname)
    env = gym.make('push-gui-v0')
    env.reset()

    for a in actions:
        #a = np.array([3.307, -30.288,  -4.121,  -2.636,   0.478,   0.177,  -0.004])
        # a = 1 * np.ones(7)
        _, _, done, _ = env.step(a)
        print(a)
        print(done)
        # time.sleep(0.1)

