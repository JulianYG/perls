from bullet.control.sockets.sock import Socket 
from Queue import Queue as q
import redis, sys
from bullet.util import _START_HOOK, _SHUTDOWN_HOOK, _RESET_HOOK

class RedisSocket(Socket):

    def __init__(self, server_addr, buffer_size=4096, port=6379, db=0):

        # Need to define terminal in each different hub implementation
        self.ip = server_addr
        self.terminal = redis.StrictRedis(host=server_addr, port=port, db=db)
        self.connected_with_server = False
        self.connected_with_client = False
        self.threads = []
        self.pubsub = self.terminal.pubsub()
        self.event_queue = q(buffer_size)
        self.signal_queue = q(buffer_size)

    def broadcast_to_client(self, message):
        return self.terminal.publish('signal_channel', message)

    def broadcast_to_server(self, message):
        return self.terminal.publish('event_channel', message)

    def listen_to_client(self):
        events = []
        # TODO: Check if / while, which one is better
        while not self.event_queue.empty():
            events.append(self.event_queue.get())
        return events

    def listen_to_server(self):
        events = []
        while not self.signal_queue.empty():
            events.append(self.signal_queue.get())
        return events

    def connect_with_client(self):

        self.pubsub.subscribe(**{'event_channel': self._event_handler})

        # Start thread
        client_thread = self.pubsub.run_in_thread(sleep_time=0.001)
        self.threads.append(client_thread)

        # Send reset and load env signal
        print('Waiting for client\'s response...')
        while 1:
            if self.broadcast_to_client(_START_HOOK) > 0:
                print('Connected with client.')
                self.connected_with_client = True
                break
        return 0

    def connect_with_server(self):

        self.pubsub.subscribe(**{'signal_channel': self._signal_handler})
        server_thread = self.pubsub.run_in_thread(sleep_time=0.001)
        self.threads.append(server_thread)

        print('Waiting for server\'s response...')
        while 1:
            if self.broadcast_to_server(_RESET_HOOK) > 0:
                print('Connected with server on {}'.format(self.ip))
                self.connected_with_server = True
                break
        return 0

    def _event_handler(self, msg):
        packet = msg['data']
        if isinstance(packet, str) or isinstance(packet, bytes):
            data = eval(packet)
            if not self.event_queue.full():
                self.event_queue.put(data)

    def _signal_handler(self, msg):
        packet = msg['data']
        if isinstance(packet, str) or isinstance(packet, bytes):
            data = eval(packet)
            if not self.signal_queue.full():
                self.signal_queue.put(data)

    def disconnect(self):
        if self.connected_with_server:
            self.broadcast_to_server(_SHUTDOWN_HOOK)
            self.connected_with_server = False
        if self.connected_with_client:
            self.broadcast_to_client(_SHUTDOWN_HOOK)
            self.connected_with_client = False
        self.pubsub.unsubscribe()
        for t in self.threads:
            t.stop()



