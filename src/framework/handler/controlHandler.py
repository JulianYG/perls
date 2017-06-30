from .base import InterruptHandler
import time
from ..utils import math_util
from ..utils import event_listener
from ..utils import network


class CmdEventHandler(InterruptHandler):
    """
    For algorithmic learning usage, such as 
    passing commands into gym environment.
    """
    pass


class KeyboardEventHandler(InterruptHandler):
    """
    Handler for keyboard events/signal
    """
    def __init__(self, sensitivity, rate=100):
        super(KeyboardEventHandler, self).__init__(rate)
        self._sens = sensitivity

        # Keep a private orientation vec to simulate
        # effect of absolute orientation
        self._orn = math_util.zero_vec(3)

    @property
    def signal(self):
        """
        Get the signal read from interruption.
        Note keyboard event only gives high level 
        instructions reach and grasp
        :return: List of signals
        """
        self._signal['cmd'] = list()
        ins = list()

        events = event_listener.listen_to_keyboard()
        time.sleep(1. / self._rate)
        for long_key, const in events.items():
            key = int(long_key)
            label = event_listener.KEY_LABEL.get(key, None)

            status = event_listener.KEY_STATUS[int(const)]

            # Using multiple if's to allow simultaneous key pressing
            if key in range(48, 58) and status == 'releasing':
                # 'id' key_type
                self._signal['tid'] = int(key) - 48
            if label == 'tool' and status == 'releasing':
                self._signal['tid'] += event_listener.HOT_KEY[key]
            if label == 'key' and status == 'releasing':
                self._signal[label] = event_listener.HOT_KEY[key]
            if label == 'rst' and status == 'pressing':
                # TODO: F5 for reset
                pass

            if label == 'grasp' and status == 'releasing':
                ins.append((label, -1))

            if label == 'pos' and status == 'holding':
                ins.append(
                    ('reach', (event_listener.HOT_KEY[key] * self._sens, None)))
            if label == 'orn' and status == 'holding':
                self._orn += event_listener.HOT_KEY[key] * self._sens

                # Don't touch position, only orientation
                ins.append(('reach', (math_util.zero_vec(3),
                                      math_util.euler2quat(self._orn))))
        self._signal['instruction'] = ins
        return self._signal


class ViveEventHandler(InterruptHandler):

    pass


class AppEventHandler(InterruptHandler):
    """
    Handler for keyboard events/signal
    """

    def __init__(self, ip='localhost', port=6379, rate=100):
        super(AppEventHandler, self).__init__(rate)
        # Use relative position
        self._internal_pos = math_util.zero_vec(3)
        # But use absolute orientation
        self._internal_orn = math_util.zero_vec(3)

        self._comm = network.RedisComm(ip, port)

    @property
    def signal(self):
        self._signal['cmd'] = list()
        ins = list()

        events = event_listener.listen_to_redis(self._comm.get_channel('iphone'))
        time.sleep(1. / self._rate)

        for event in events:
            roll, pitch, yaw, sens = [float(x) for x in event.split(' ')]
            self._internal_orn = math_util.zero_vec(3)


            orn_euler = np.array([roll, -pitch, yaw],
                                 dtype=np.float32) * np.pi / 180.
            if not agent.FIX:
                pseudo_event[2] = p.getQuaternionFromEuler(
                    np.arcsin(np.sin(orn_euler * sens)))
            end_effector_poses = agent.get_tool_poses(tools)

            self.pos = end_effector_poses[:, 0]
            pseudo_event[1] = self.pos[pseudo_event[0]]

            agent.control(pseudo_event, control_map)

        self._signal['instruction'] = ins
    
    