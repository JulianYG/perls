#!/usr/bin/env python

# Intera SDK Wrapper for use w/ PyBullet

import redis 
import time
import pybullet as p

p.connect(p.SHARED_MEMORY)
r = redis.StrictRedis(host='172.24.68.111', port=6379)

while True:
    events = p.getVREvents()
    for e in (events):
        r.publish('event_channel', e)
