import sys
import time
import openvr
import redis
from bullet.models import *
from bullet.control import *
from bullet.simulator import BulletSimulator
import os, sys, getopt, json
from os.path import join as pjoin
import bullet.util as utils
from bullet.control.hub import *
import json
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

ip = '172.24.68.111'
model = pr2.PR2([0.3, -0.5], enableForceSensor=False)
host = hub.redis_hub.RedisServer(ip)
interface = vr_interface.IVR(host, False)

REPO_DIR = pjoin(os.getcwd(), 'data', 'task.json')
with open(REPO_DIR, 'r') as f:
	repo = json.loads(f.read())
task = repo['ball']

simulator = BulletSimulator(model, interface, task, True)

simulator.run(remote_render=True)


# def cmd_handler(msg):
# 	data = eval(msg)

# 	if data == 0:
# 		simulator.quit()
# 	elif data == 1:
# 		simulator.setup(task, 0, True)
# 	else:
# 		for obj, pose in data.items():
# 			p.resetBasePositionAndOrientation(obj, pose)

# r = redis.StrictRedis(host=host, port=6379, db=0)
# subscriber = r.pubsub()

# subscriber.subscribe(**{'client_channel': cmd_handler})

# p.connect(p.SHARED_MEMORY)
# while True:
# 	event = p.getVREvents()

# 	for e in (event):
# 		x = r.publish('server_channel', e)

# 	time.sleep(0.001)
# 		# iD = e[0]
# 		# pos = e[1]
# 		# orn = e[2]


