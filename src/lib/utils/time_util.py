import time
from threading import Thread, Event


class Timer(Thread):

    def __init__(self, interval, func, max_iter, *args):

        super(Timer, self).__init__()
        self._interval = interval
        self._func = func
        self._args = args
        self._max_iter = max_iter
        self._done = Event()
        self.daemon = True

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self._done.set()

    def run(self):
        if self._max_iter:
            for _ in range(self._max_iter):
                if self._done.is_set():
                    break
                old_time = time.time()
                self._func(*self._args)
                interval = time.time() - old_time
                if self._interval > interval:
                    time.sleep(self._interval - interval)
        else:
            while 1 and not self._done.is_set():
                old_time = time.time()
                self._func(*self._args)
                interval = time.time() - old_time
                if self._interval > interval:
                    time.sleep(self._interval - interval)


def pause(t):
    time.sleep(t)


def get_abs_time():

    return time.time()


def get_time_stamp():
    return time.strftime('%H:%M:%s', time.localtime())


def get_full_time_stamp():

    return time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime())


def get_elapsed_time(start=None):

    if start > 0:
        # Pack into some human readable form
        return time.time() - start
    else:
        return 0.

