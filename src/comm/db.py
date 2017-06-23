__package__ = 'comm'

import redis, sys
if sys.version[0] == '2':
    from Queue import Queue as q
else:
    from queue import Queue as q

from .core import Comm

from bullet_.simulation.utils.misc import Constant

class RedisComm(Comm):

    def __init__(self, server_addr, buffer_size=4096, port=6379, db=0):

        # Need to define terminal in each different hub implementation
        self.ip = server_addr
        self.terminal = redis.StrictRedis(host=server_addr, port=port, db=db)
        self.connected_with_server = False
        self.connected_with_client = False
        self.threads = []
        self._buffer_size = buffer_size
        self.pubsub = self.terminal.pubsub()
        self.channel_queues = dict(event_channel=q(buffer_size),
            signal_channel=q(buffer_size))

    def broadcast_to_channel(self, channel, message):
        return self.terminal.publish(channel, message)

    def broadcast_to_client(self, message):
        return self.terminal.publish('signal_channel', message)

    def broadcast_to_server(self, message):
        return self.terminal.publish('event_channel', message)

    def listen_to_channel(self, channel):
        events = []
        queue = self.channel_queues[channel]
        while not queue.empty():
            events.append(queue.get())
        print(events)
        return events

    def connect_to_channel(self, channel):
        self.channel_queues[channel] = q(self._buffer_size)
        self.pubsub.subscribe(**{channel: self._channel_handler})
        # Start thread
        channel_thread = self.pubsub.run_in_thread(sleep_time=0.001)
        self.threads.append(channel_thread)

    def connect_with_client(self, channel='event_channel'):
        if not self.connected_with_client:
            self.pubsub.subscribe(**{channel: self._channel_handler})

            # Start thread
            client_thread = self.pubsub.run_in_thread(sleep_time=0.001)
            self.threads.append(client_thread)

            # Send reset and load env signal
            print('Waiting for client\'s response...')
            while 1:
                if self.broadcast_to_client(Constant.START_HOOK) > 0:
                    print('Connected with client.')
                    self.connected_with_client = True
                    break

    def connect_with_server(self, channel='signal_channel'):
        if not self.connected_with_server:
            self.pubsub.subscribe(**{channel: self._channel_handler})
            server_thread = self.pubsub.run_in_thread(sleep_time=0.001)
            self.threads.append(server_thread)

            print('Waiting for server\'s response...')
            while 1:
                if self.broadcast_to_server(Constant.RESET_HOOK) > 0:
                    print('Connected with server on {}'.format(self.ip))
                    self.connected_with_server = True
                    break

    def _channel_handler(self, msg):
        packet = msg['data']
        channel_queue = self.channel_queues[msg['channel']]
        if isinstance(packet, str) or isinstance(packet, bytes):
            if not channel_queue.full():
                channel_queue.put(packet)

    def disconnect(self):
        if self.connected_with_server:
            self.broadcast_to_server(Constant.SHUTDOWN_HOOK)
            self.connected_with_server = False
        if self.connected_with_client:
            self.broadcast_to_client(Constant.SHUTDOWN_HOOK)
            self.connected_with_client = False
        for t in self.threads:
            t.stop()
        self.pubsub.unsubscribe()


