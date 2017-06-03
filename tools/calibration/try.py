#!/usr/bin/env python

import rospy
import tf
import numpy as np
rospy.init_node('xx')

br = tf.TransformBroadcaster()
tl = tf.TransformListener()
import time

time.sleep(2)


c = 0

trans = []
rots = []

while not rospy.is_shutdown():
	tran, rot = tl.lookupTransform('kinect2_rgb_optical_frame', 'chessboard', tl.getLatestCommonTime('kinect2_rgb_optical_frame', 'chessboard'))
	#tran, rot = tl.lookupTransform('chessboard', 'kinect2_rgb_optical_frame', tl.getLatestCommonTime('kinect2_rgb_optical_frame', 'chessboard'))

	Rmat = tf.transformations.quaternion_matrix(rot)


	rm1 = tf.transformations.rotation_matrix(np.pi / 2, [0, 0, 1])
	rm2 = tf.transformations.rotation_matrix(np.pi, [1, 0, 0])

	rm = rm1.dot(rm2)

	Rmat = Rmat.dot(rm)


	TR = Rmat.copy()
	TR[:3, 3] += tran
	offset = np.eye(4)
	offset[0, 3] += 0.05
	offset[1, 3] += 0.11
	offset[2, 3] -= 0.02
	TR = TR.dot(offset)
	tran = TR[:3, 3]

	invRmat = np.linalg.inv(Rmat[:3, :3])
	d = -invRmat.dot(np.array(tran))

	Rmat[:3,:3] = invRmat

	nQ = tf.transformations.quaternion_from_matrix(Rmat)
	br.sendTransform(d, nQ, 
					rospy.Time.now(),  '/kinect2_rgb_optical_frame', 'right_hand')
	if c > 100 and c < 200:
		tran, rot = tl.lookupTransform('base', 'kinect2_rgb_optical_frame', tl.getLatestCommonTime('kinect2_rgb_optical_frame', 'base'))
		trans.append(np.array(tran))
		rots.append(np.array(rot))
	if c == 200:
		tmean = np.mean(trans, axis=0)
		rmean = np.mean(rots, axis=0)
		rmean = rmean / np.sqrt(np.sum(rmean ** 2))
		print(tmean, 'tmean')
		print(rmean, 'rmean')
		trans = []
		rots = []
		c = 0
	c += 1
	rospy.Rate(10.).sleep()
