import time
import datetime


def get_time_stamp():

    return time.time()


def get_elapsed_time(start=None):

    if start > 0:
        # Pack into some human readable form
        return time.time() - start
    else:
        return 0.

