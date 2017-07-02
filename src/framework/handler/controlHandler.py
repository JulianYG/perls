from .base import InterruptHandler
import time
from ..utils import math_util
from ..utils import event_listener
from ..utils import network

_EVENT_LABEL = {
    0: 'key',  # Alphabet 'g' for grippers, 'm' for arms
    1: 'id',  # integer 0-9 to label tools
    2: 'rst',  # integer 1 for reset, 0 for nothing
    3: 'grasp',  # 1 for grasp, 0 for release; or float
                 # in between to slide (3d-touch)
    4: 'pos',  # vec3 float cartesian relative
    5: 'orn',  # vec3 float orientation absolute
}


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
        self._sig_orn = math_util.zero_vec(3)

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

        events = event_listener.listen_to_bullet_keyboard()
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
                ins.append((label, 1))
            if label == 'grasp' and status == 'releasing':
                ins.append((label, -1))
            if label == 'pos' and status == 'holding':
                ins.append(
                    ('reach', (event_listener.HOT_KEY[key] * self._sens, None)))
            if label == 'orn' and status == 'holding':
                self._sig_orn += event_listener.HOT_KEY[key] * self._sens

                # Don't touch position, only orientation (quat)
                ins.append(('reach', (None, math_util.euler2quat(self._sig_orn))))
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
        self._comm = network.RedisComm(ip, port)
        self._comm.connect_to_channel('redis')

    @property
    def signal(self):
        self._signal['cmd'] = list()
        ins = list()

        events = event_listener.listen_to_redis(
            self._comm.channels['redis'])
        time.sleep(1. / self._rate)

        for label_id, value in events:

            label = _EVENT_LABEL[label_id]
            if label_id < 4:
                ins.append((label, value))
            elif label_id == 4:
                ins.append(('reach', (value, None)))
            elif label_id == 5:
                # Scale by sensitivity
                orn = value[:3] * value[3]
                ins.append(('reach', (math_util.zero_vec(3), orn)))

        self._signal['instruction'] = ins
        return self._signal
