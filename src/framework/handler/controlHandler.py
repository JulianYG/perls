from .base import InterruptHandler
import time
from ..utils import math_util
from ..utils import event_listener


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
    def __init__(self, sensitivity, rate=1000):
        InterruptHandler.__init__(self, sensitivity, rate)
        # Use relative position
        self._internal_pos = math_util.zero_vec(3)
        # But use absolute orientation
        self._internal_orn = math_util.zero_vec(3)

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
                ins.append((label, None))
            if label == 'pos' and status == 'holding':
                self._internal_pos += event_listener.HOT_KEY[key] * self._sens
                ins.append(('reach', (self._internal_pos,
                                      math_util.euler2quat(self._internal_orn))))
            if label == 'orn' and status == 'holding':
                self._internal_orn += event_listener.HOT_KEY[key] * self._sens
                ins.append(('reach', (math_util.zero_vec(3),
                                      math_util.euler2quat(self._internal_orn))))

        self._signal['instruction'] = ins
        return self._signal

    def _map_pos(self, pos):

        pass

    def _map_orn(self, orn):

        pass

class ViveEventHandler(InterruptHandler):

    pass


class AppEventHandler(InterruptHandler):

    pass
