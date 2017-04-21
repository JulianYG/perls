import sys
import time
import openvr
import redis
from node import build
# openvr.init(openvr.VRApplication_Scene)

# poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
# poses = poses_t()

# print(type(poses_t))
# print(type(poses))
# while 1:
#     openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
#     hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
#     print(hmd_pose.mDeviceToAbsoluteTracking)
#     # sys.stdout.flush()
#     time.sleep(1)

# openvr.shutdown()


import pybullet as p
host = '172.24.68.111'

simulator = build(model, task, socket, filename, record=False)

def cmd_handler(msg):
	data = eval(msg)

	if data == 0:
		shutdown
	elif data == 1:
		reset
	else:
		pass


r = redis.StrictRedis(host=host, port=6379, db=0)
subscriber = r.pubsub()

subscriber.subscribe(**{'client_channel': cmd_handler})

p.connect(p.SHARED_MEMORY)
while True:
	event = p.getVREvents()

	for e in (event):
		x = r.publish('server_channel', e)

	time.sleep(0.001)
		# iD = e[0]
		# pos = e[1]
		# orn = e[2]


