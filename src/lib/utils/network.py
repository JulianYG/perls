import redis
import sys
if sys.version[0] == '2':
    from queue import Queue
else:
    from Queue import Queue


class RedisComm(object):

    def __init__(self, server_addr, buffer_size=1024, port=6379, db=0):

        # Need to define terminal in each different hub implementation
        self.ip = server_addr
        self.terminal = redis.StrictRedis(host=server_addr, port=port, db=db)

        self.threads = []
        self._buffer_size = buffer_size

        self.pubsub = self.terminal.pubsub()

        self.channel_queues = dict()

    @property
    def channels(self):
        """
        Get all channels this server is currently
        connected to
        :return: dictionary where keys are channel names,
        and values are the queues.
        """
        return self.channel_queues

    def broadcast_to_channel(self, channel, message):
        return self.terminal.publish(channel, message)

    def listen_to_channel(self, channel):
        events = []
        queue = self.channel_queues[channel]
        while not queue.empty():
            events.append(queue.get())
        # print(events)
        return events

    def connect_to_channel(self, channels):
        if not isinstance(channels, list):
            channels = [channels]
        for channel in channels:
            self.channel_queues[channel] = Queue(self._buffer_size)
            self.pubsub.subscribe(**{channel: self._channel_handler})
            # Start thread
            channel_thread = self.pubsub.run_in_thread(sleep_time=0.001)
            self.threads.append(channel_thread)

    def _channel_handler(self, msg):
        packet = msg['data']
        channel_name = msg['channel']
        if isinstance(msg['channel'], bytes):
            channel_name = msg['channel'].decode('utf-8')
        channel_queue = self.channel_queues[channel_name]
        if isinstance(packet, str) or isinstance(packet, bytes):
            if not channel_queue.full():
                channel_queue.put(packet)

    def disconnect(self):
        for t in self.threads:
            t.stop()
        self.pubsub.unsubscribe()


