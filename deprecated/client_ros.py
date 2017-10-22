#!/usr/bin/env python

# Intera SDK Wrapper for use w/ PyBullet

import redis 
import time

from perls import controlHandler
from multiprocessing import Queue

def get_registered_device()


if __name__ == '__main__':

    queue = Queue()
    r = redis.StrictRedis(host='charles.stanford.edu', port=6379)
    vr = controlHandler.ViveEventHandler(0, queue, 5)

    
    while True:

        try:
            if not queue.empty():
                signal = queue.get_nowait()

                print(signal)




        except KeyboardInterrupt:
            vr.close()
