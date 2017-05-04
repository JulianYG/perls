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

class Robot_Control(object):

	# Takes a limb and gripper instance as args
	def __init__(self, limb, gripper):

		rs = intera_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		rs.enable() # enable all joints

		rospy.loginfo("Robot Enabled")
		self.limb = limb
		self.gripper = gripper

		self.initEndpointPose = self.limb.endpoint_pose() # initial pose of end effector

	# For LARGE MOVEMENTS: move joints to desired angles (motion planner)
	# Takes in a dictionary (string:float) with entries for each joint on the robot
	# Ex: {'right_j6': -0.2, 'right_j5': -1.0, 'right_j4': -1.8, 'right_j3': 1.4, 'right_j2': -1.0, 'right_j1': -0.2, 'right_j0': 0.9}
	def moveJointStates(self, angles):
		self.limb.move_to_joint_positions(angles) # threshold of .0087 radians for when end state has been reached
		rospy.loginfo("Moved arm to desired position")

	# Takes in a dictionary (string:float) with entries for each joint on the robot
	def setJointPositions(self, angles): 
		self.limb.set_joint_positions(angles) # look into the raw bool flag -> commands joint pos without modification to JCB -> bypasses safety check
		rospy.loginfo("Moved arm to desired position")

	def getJointAngles(self):
		self.jointAngles = self.limb.joint_angles()
		return self.jointVelocities

	def getJointVelocities(self):
		self.jointVelocities = self.limb.joint_velocities()
		return self.jointVelocities

	def getJointEfforts(self):
		self.jointEfforts = self.limb.joint_efforts()
		return self.jointEfforts

	def getEndpoint(self):
		self.currEndpoint = self.limb.endpoint_pose()
		return self.currEndpoint

	# Takes in a dict of {'position' : (x,y,z), 'orientation' : (x,y,z,w)} and uses inverse IK solver to map and set the joint position
	def setReach(self, endState): 
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
			rospy.loginfo("Move to position succeeded")

		else:
			rospy.logerr("IK response is not valid")



	# Takes in a float val from 0.0 to 1.0 and maps that to the gripper
	def setGripper(self, gripperVal):
		scaledGripPos = gripperVal * self.gripper.MAX_POSITION
		self.gripper.set_position(scaledGripPos)

	# Get the relative endpoint cartesian coordinates and orientation as compared to the arm's initial state
	def getRelativePos(self):
		currentEndpoint = self.getEndpoint()

		currPos = currentEndpoint['position']
		currOrient = currentEndpoint['orientation']
		origPos = self.initEndpointPose['position']
		origOrient = self.initEndpointPose['orientation']

		diffPos = self.limb.Point._make(x-y for x,y in zip(currPos, origPos))
		diffOrient = self.limb.Quaternion._make(x-y for x,y in zip(currOrient, origOrient))

		relPos = {'position' : diffPos, 'orientation' : diffOrient}
		return relPos

		

