#!/usr/bin/env python

import perls

# Prevent recursively spawning sub-processes
if __name__ == '__main__':

    s = perls.Controller('config.xml')
    # s.start_all()

    s.start()
