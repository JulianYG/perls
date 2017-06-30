import time


def get_abs_time():

    return time.time()

def get_time_stamp():

    return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())

def get_elapsed_time(start=None):

    if start > 0:
        # Pack into some human readable form
        return time.time() - start
    else:
        return 0.

