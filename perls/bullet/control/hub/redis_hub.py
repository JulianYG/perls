from bullet.control.hub.hub import Hub
from queue import Queue as q
import redis, sys
from bullet.util import *

class RedisServer(Hub):

    def __init__(self, server_addr, buffer_size=4096, port=6379, db=0):

        # Need to define terminal in each different hub implementation
        self.terminal = redis.StrictRedis(host=server_addr, port=6379, db=db)
        self.connected = False
        self.pubsub = self.terminal.pubsub()
        self.event_queue = q(buffer_size)

    def broadcast_msg(self, message):
        return self.terminal.publish('client_channel', message)

    def read_msg(self, *args):
        events = []
        if not self.event_queue.empty():
            events.append(self.event_queue.get())
        return events

    def connect(self):
        # How many channels do we have
        self.pubsub.subscribe(**{'server_channel': self._event_handler})

        # Start another thread to listen for events
        self.thread = self.pubsub.run_in_thread(sleep_time=0.001)

        # Send reset and load env signal
        if self.broadcast_msg(_START_HOOK) > 0:
            self.connected = True

    def _event_handler(self, msg):
        data = msg['data']
        if isinstance(data, str) or isinstance(data, bytes):
            # if isinstance(packet, list):
            #     self.model.set_virtual_controller(packet)
            # elif packet == _SHUTDOWN_HOOK:
            #     sys.exit(0)
            # else:
            if not self.event_queue.full():
                self.event_queue.put(eval(data))

    def close(self):
        self.connected = False
        self.broadcast_msg(_SHUTDOWN_HOOK)
        self.pubsub.unsubscribe()


