#!/usr/bin/env python

import rospy
import tf
import numpy as np
rospy.init_node('publisher')

br = tf.TransformBroadcaster()
tl = tf.TransformListener()
import time

time.sleep(1)

trans = []
rots = []

for _ in xrange(10):
	tran, rot = tl.lookupTransform('base', 'kinect2_rgb_optical_frame', tl.getLatestCommonTime('kinect2_rgb_optical_frame', 'base'))
	trans.append(np.array(tran))
	rots.append(np.array(rot))
	raw_input('next?')

tmean = np.mean(trans)
rmean = np.mean(rots)
rmean = rmean / np.sqrt(np.sum(rmean ** 2))
print(tmean, 'tmean')
print(rmean, 'rmean')