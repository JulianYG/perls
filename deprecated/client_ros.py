#!/usr/bin/env python

# Intera SDK Wrapper for use w/ PyBullet

import redis 
import time
import pybullet as p

p.connect(p.SHARED_MEMORY)
r = redis.StrictRedis(host='charles.stanford.edu', port=6379)

while True:
    cur = time.time()
    events = p.getVREvents()
    for e in (events):
        r.publish('event_channel', e)
    diff = time.time() - cur
    time.sleep(0.2 - diff)