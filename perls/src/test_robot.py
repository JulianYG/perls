import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from ros_.robot import Robot
import time
from utils import Interpolator
import numpy as np
import cPickle

LOAD = True
RECORD = False
ENDPOINTS = False

rospy.init_node('sdk_wrapper')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')


rest_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
	         'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

inter_pose = {'right_j6': 3.3161, 'right_j5': 0.57, 'right_j4': 0, 
             'right_j3': 2.18, 'right_j2': -0, 'right_j1': -1.18, 'right_j0': 0.}

new_pose = {'right_j6': 3.3142109375, 'right_j5': 0.5868447265625, 'right_j4': -0.029119140625, 
            'right_j3': -0.2258154296875, 'right_j2': 0.3116494140625, 'right_j1': -1.0423837890625, 
            'right_j0': 0.9351689453125}

sorted_keys = sorted(rest_pose)

limb.move_to_neutral()

def get_joint_vector(joint_dict):
    return np.array(list(map(joint_dict.get, sorted_keys)))

joint_bounds = np.array([0.002, 0.002, 0.002, 0.002, 0.002, 0.002, 0.003]) 

interp = Interpolator()
initial = get_joint_vector(rest_pose)
final = get_joint_vector(new_pose)
pts = interp.dynamic_linear_interpolation(initial, final, joint_bounds)

num_pts, num_joints = pts.shape

arm = Robot(limb, gripper)

rate = rospy.Rate(100) # Try to make it 100 Hz
threshold = 0.008726646

if ENDPOINTS:
	# pts = [pts[0], pts[len(pts)/2], pts[-1]]
	pts = [pts[0], pts[-1]]
	num_pts = len(pts)

trajectory = []
velocities = []

if LOAD:
	f = open('fw_trajectory', 'rb')
	pts = cPickle.load(f)
	num_pts = len(pts)
	f.close()

	f = open('fw_velocity', 'rb')
	v_pts = cPickle.load(f)
	assert len(v_pts) == num_pts
	f.close()

for i in range(num_pts):

    target = dict(zip(sorted_keys, pts[i]))
    current = get_joint_vector(limb._joint_angle)
    cur_v = get_joint_vector(limb._joint_velocity)
    trajectory.append(current)
    velocities.append(cur_v)

    while not np.allclose(pts[i], current, rtol=0.0, atol=threshold):
    	print(i, 'fw')

    	# arm.set_joint_positions(target)
    	limb.set_joint_trajectory(sorted_keys,
    		pts[i], v_pts[i], [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
    	# if i < len(v_pts) - 1:
    	# 	limb.set_joint_velocities(dict(zip(sorted_keys, v_pts[i])))
    	# else:
    	# 	limb.set_joint_velocities(dict(zip(sorted_keys, [0] * 7)))

    	current = get_joint_vector(limb._joint_angle)
    	cur_v = get_joint_vector(limb._joint_velocity)
    	trajectory.append(current)
    	velocities.append(cur_v)
    	rate.sleep()

    # def genf(joint, angle):
    #     def joint_diff():
    #         return abs(angle - limb._joint_angle[joint])
    #     return joint_diff

    # diffs = [genf(j, a) for j, a in target.items()]

    # arm.set_joint_positions(target)

    # while not (all(diff() < threshold for diff in diffs)):
    #     print(i)
    #     arm.set_joint_positions(target)
    #     rate.sleep()

    # arm.plan_joint_positions(target)

if RECORD:
	f = open('fw_trajectory', 'wb')
	cPickle.dump(trajectory, f)
	f.close()

	f = open('fw_velocity', 'wb')
	cPickle.dump(velocities, f)
	f.close()

pts = interp.dynamic_linear_interpolation(final, initial, joint_bounds)

if ENDPOINTS:
	# pts = [pts[0], pts[len(pts)/2], pts[-1]]
	pts = [pts[0], pts[-1]]
	num_pts = len(pts)

trajectory2 = []
velocities2 = []

if LOAD:
	f = open('bw_trajectory', 'rb')
	pts = cPickle.load(f)
	num_pts = len(pts)
	f.close()

	f = open('bw_velocity', 'rb')
	v_pts = cPickle.load(f)
	f.close()

for i in range(num_pts):
    print(i)
    target = dict(zip(sorted_keys, pts[i]))
    current = get_joint_vector(limb._joint_angle)
    cur_v = get_joint_vector(limb._joint_velocity)
    trajectory2.append(current)
    velocities2.append(cur_v)

    while not np.allclose(pts[i], current, rtol=0.0, atol=threshold):
    	print(i, 'bw')


    	# arm.set_joint_positions(target)
    	limb.set_joint_trajectory(sorted_keys,
    		pts[i], v_pts[i], [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001, -0.001])


    	current = get_joint_vector(limb._joint_angle)
    	cur_v = get_joint_vector(limb._joint_velocity)
    	trajectory2.append(current)
    	velocities2.append(cur_v)
    	rate.sleep()

    
    # def genf(joint, angle):
    #     def joint_diff():
    #         return abs(angle - limb._joint_angle[joint])
    #     return joint_diff

    # diffs = [genf(j, a) for j, a in req_joint_angles.items()]

    # arm.set_joint_positions(req_joint_angles)

    # while not (all(diff() < threshold for diff in diffs)):
    #     print(i)
    #     arm.set_joint_positions(req_joint_angles)
    #     rate.sleep()

    # arm.plan_joint_positions(target)
    # print(i)

if RECORD:
	f = open('bw_trajectory', 'wb')
	cPickle.dump(trajectory2, f)
	f.close()

	f = open('bw_velocity', 'wb')
	cPickle.dump(velocities2, f)
	f.close()

print(len(trajectory))
print(len(trajectory2))

print(np.mean(np.array(velocities), axis=0))
print(np.mean(np.array(velocities2), axis=0))

# print(arm.get_joint_angles())

# arm.plan_joint_positions(rest_pose)
# # time.sleep(1.0)
# arm.plan_joint_positions(new_pose)
# # time.sleep(1.0)
# arm.plan_joint_positions(rest_pose)