from bullet.control.hub.hub import Hub
from Queue import Queue as q
import redis, sys

class RedisServer(Hub):

    def __init__(self, server_addr, buffer_size=4096, port=6379, db=0):

        super(RedisServer, self).__init__()

        # Need to define terminal in each different hub implementation
        self.terminal = redis.StrictRedis(host=server_addr, port=6379, db=db)
        self.connected = False
        self.pubsub = self.terminal.pubsub()
        self.event_queue = q(buffer_size)

    def broadcast_msg(self, message):
        self.terminal.publish('client_channel', message)
        # pass
    def read_msg(self, *args):
        events = []
        if not self.event_queue.empty():
            events.append(self.event_queue.get())
        return events

    def connect(self, model):
        self.model = model
        # How many channels do we have
        self.pubsub.subscribe(**{'server_channel': self._event_handler})

        # Start another thread to listen for events
        self.thread = self.pubsub.run_in_thread(sleep_time=0.001)

        # Send reset and load env signal
        if self.broadcast_msg(Hub._RESET_HOOK) > 0:
            self.connected = True

    def _event_handler(self, msg):
        data = msg['data']
        if isinstance(data, str) or isinstance(data, bytes):
            packet = eval(data)
            if isinstance(packet, list):
                self.model.set_virtual_controller(packet)
            elif packet == Hub._SHUT_DOWN_HOOK:
                sys.exit(0)
            else:
                if not self.event_queue.full():
                    self.event_queue.put(packet)

    def close(self):
        self.connected = False
        self.broadcast_msg(Hub._SHUT_DOWN_HOOK)
        self.pubsub.unsubscribe()


