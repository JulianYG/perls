#!/usr/bin/env python

import ccr

# Prevent recursively spawning sub-processes
if __name__ == '__main__':

    s = ccr.Controller('config.xml')
    # s.start_all()

    s.start()
