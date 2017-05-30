#!/usr/bin/env python
# Intera SDK Wrapper for use w/ PyBullet

import rospy
import intera_interface
from intera_interface import CHECK_VERSION

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
	SolvePositionFK,
	SolvePositionFKRequest
)
'''
Before initializing an instance of the Robot_Control class:
	rospy.init_node("sdk_wrapper") #initializes node on machine to talk to arm
	limb = intera_interface.Limb('right')
	gripper = intera_interface.Gripper('right')
	arm = Robot_Control(limb, gripper)
	
'''

class Robot(object):

	# Takes a limb and gripper instance as args
	def __init__(self, limb, gripper):

		rs = intera_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		rs.enable() # enable all joints

		rospy.loginfo("Robot Enabled")
		self.limb = limb
		self.gripper = gripper

		self.initEndpointPose = None

	def set_init_positions(self, position):
		
		self.plan_joint_positions(position)
		self.initEndpointPose = self.limb.endpoint_pose() # initial pose of end effector

	# For LARGE MOVEMENTS: move joints to desired angles (motion planner)
	# Takes in a dictionary (string:float) with entries for each joint on the robot
	# Ex: {'right_j6': -0.2, 'right_j5': -1.0, 'right_j4': -1.8, 'right_j3': 1.4, 'right_j2': -1.0, 'right_j1': -0.2, 'right_j0': 0.9}
	def plan_joint_positions(self, angles):
		self.limb.move_to_joint_positions(angles) # threshold of .0087 radians for when end state has been reached
		rospy.loginfo("Moved arm to desired position")

	# Takes in a dictionary (string:float) with entries for each joint on the robot
	def set_joint_positions(self, angles): 
		self.limb.set_joint_positions(angles) # look into the raw bool flag -> commands joint pos without modification to JCB -> bypasses safety check
		rospy.loginfo("Moved arm to desired position")

	def get_joint_angles(self):
		return self.limb.joint_angles()

	def get_joint_velocities(self):
		return self.limb.joint_velocities()

	def get_joint_efforts(self):
		return self.limb.joint_efforts()

	def get_tool_pose(self):
		pose = self.limb.endpoint_pose()
		pos, orn = pose['position'], pose['orientation']
		return (pos.x, pos.y, pos.z), (orn.w, orn.x, orn.y, orn.z)

	# Takes in a dict of {'position' : (x, y, z), 'orientation' : (w, x, y, z)} and uses inverse IK solver to map and set the joint position
	def reach_absolute(self, endState): 
		ns = "ExternalTools/right/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		poses = {
			'right': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(*(endState['position'])),
					orientation=Quaternion(*(endState['orientation'])),
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
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False

		if (resp.result_type[0] > 0):
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position)) # format IK response for compatability with Limb
			self.limb.move_to_joint_positions(limb_joints)
			# print(limb_joints)
			# self.limb.set_joint_positions(limb_joints)
			rospy.loginfo("Move to position succeeded")

		else:
			rospy.logerr("IK response is not valid")
			return False

	# def reach_relative(self, endState):
	def shutdown(self):
		# def clean_shutdown():
		# 	rs.disable()
		# 	print("\nExiting example...")
		rospy.signal_shutdown('Exiting example...')

	# Takes in a float val from 0.0 to 1.0 and maps that to the gripper
	def slide_grasp(self, gripperVal):
		scaledGripPos = gripperVal * self.gripper.MAX_POSITION
		self.gripper.set_position(scaledGripPos)

	# Get the relative endpoint cartesian coordinates and orientation as compared to the arm's initial state
	def get_relative_pose(self):
		currentEndpoint = self.get_tool_pose()

		currPos = currentEndpoint[0]
		currOrient = currentEndpoint[1]
		origPos = self.initEndpointPose['position']
		origOrient = self.initEndpointPose['orientation']

		diffPos = self.limb.Point._make(x-y for x,y in zip(currPos, origPos))
		diffOrient = self.limb.Quaternion._make(x-y for x,y in zip(currOrient, origOrient))

		relPos = {'position' : diffPos, 'orientation' : diffOrient}
		return relPos

		

