from bullet.control.sockets.sock import Socket 
from Queue import Queue as q
import redis, sys
from bullet.util import _START_HOOK, _SHUTDOWN_HOOK, _RESET_HOOK

class RedisSocket(Socket):

    def __init__(self, server_addr, buffer_size=4096, port=6379, db=0):

        # Need to define terminal in each different hub implementation
        self.ip = server_addr
        self.terminal = redis.StrictRedis(host=server_addr, port=6379, db=db)
        self.connected = False
        self.threads = []
        self.pubsub = self.terminal.pubsub()
        self.client_event_queue = q(buffer_size)
        self.server_event_queue = q(buffer_size)

    def broadcast_to_client(self, message):
        return self.terminal.publish('server_channel', message)

    def broadcast_to_server(self, message):
        return self.terminal.publish('client_channel', message)

    def listen_to_client(self):
        events = []
        if not self.client_event_queue.empty():
            events.append(self.client_event_queue.get())
        return events

    def listen_to_server(self):
        events = []
        if not self.server_event_queue.empty():
            events.append(self.server_event_queue.get())
        return events

    def connect_with_client(self):

        # Send reset and load env signal
        if self.broadcast_to_client(_START_HOOK) > 0:
            print('Connected with client.')
            self.connected = True

        # How many channels do we have
        self.pubsub.subscribe(**{'client_channel': self._client_event_handler})

        # Start thread
        self.threads.append(self.pubsub.run_in_thread(sleep_time=0.001))

    def connect_with_server(self):

        if self.broadcast_to_server(_RESET_HOOK) > 0:
            print('Connected with server on {}'.format(self.ip))
            self.connected = True

        self.pubsub.subscribe(**{'server_channel': self._server_event_handler})

        self.threads.append(self.pubsub.run_in_thread(sleep_time=0.001))

    def _client_event_handler(self, msg):
        packet = msg['data']
        if isinstance(packet, str) or isinstance(packet, bytes):
            data = eval(packet)
            # if isinstance(packet, list):
            #     self.model.set_virtual_controller(packet)
            # elif packet == _SHUTDOWN_HOOK:
            #     sys.exit(0)
            # else:
            if not self.client_event_queue.full():
                self.client_event_queue.put(data)

    def _server_event_handler(self, msg):
        packet = msg['data']
        if isinstance(packet, str) or isinstance(packet, bytes):
            data = eval(packet)
            if data == _SHUTDOWN_HOOK:
                raise SystemExit('Server invokes shut down')
            elif data == _START_HOOK:
                print('Server is online')
                # self.model.reset(0, vr)
                # self.model.setup_scene(task)
                # p.setRealTimeSimulation(0)
            else:
                if not self.server_event_queue.full():
                    self.server_event_queue.put(data)

    def close(self):
        if self.connected:
            self.broadcast_msg(_SHUTDOWN_HOOK)
            self.pubsub.unsubscribe()
            for t in self.threads:
                t.stop()
        self.connected = False


