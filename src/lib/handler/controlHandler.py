# !/usr/env/bin python

import abc

from .base import ControlHandler
from ..utils import math_util
from ..utils import event_listener
from ..utils import network

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

_EVENT_LABEL = {
    0: 'key',  # Alphabet 'g' for grippers, 'm' for arms
    1: 'id',  # integer 0-9 to label tools
    2: 'rst',  # integer 1 for reset, 0 for nothing
    3: 'grasp',  # 1 for grasp, 0 for release; or float
                 # in between to slide (3d-touch)
    4: 'pos',  # vec3 float cartesian relative
    5: 'orn',  # vec3 float orientation absolute
    6: 'camera',  # indicates camera parameter
}


class CmdEventHandler(ControlHandler):
    """
    For algorithmic learning usage, such as
    passing commands into gym environment.
    """
    def __init__(self, ps_id, queue, sensitivity=1, rate=0, qsize=200):
        super(CmdEventHandler, self).__init__(
            ps_id, queue, sensitivity, rate, qsize)

    @property
    def name(self):
        return 'CmdControl'

    def interrupt(self):
        # TODO
        pass

    def stop(self):
        return


class KeyboardEventHandler(ControlHandler):
    """
    Handler for keyboard events/signal
    """
    def __init__(self, ps_id, queue, sensitivity=1, rate=100, qsize=200):
        super(KeyboardEventHandler, self).__init__(ps_id, queue, sensitivity, rate, qsize)

    @property
    def name(self):
        return 'KeyboardControl'

    def interrupt(self, queue):

        signal = dict()

    # @property
    # def signal(self):
    #     """
    #     Get the signal read from interruption.
    #     Note keyboard event only gives high level
    #     instructions reach and grasp.
    #     :return: List of signals
    #     """
        signal['cmd'] = list()
        ins = list()
        signal['camera'] = list()
        signal['record'] = True

        events = event_listener.listen_to_bullet_keyboard(self._id)

        # Construct dictionary {label: (key, status)}
        keys = {event_listener.KEY_LABEL.get(int(long_key), None):
                    (int(long_key), event_listener.KEY_STATUS[int(const)])
                for (long_key, const) in events.items()}

        # TODO: should this use <set_camera_pose>?
        # Handle display / view settings
        if 'cam' in keys and keys['cam'][1] == 'holding':
            if 'pos' in keys and keys['pos'][1] == 'holding':

                # View control is 100 times more sensitive than robot control
                raw_delta = event_listener.HOT_KEY[keys['pos'][0]] * self._sens
                delta = math_util.vec((raw_delta[1], -raw_delta[0], raw_delta[2]))
                signal['camera'].append(('pos', delta))

            if 'orn' in keys and keys['orn'][1] == 'holding':

                # Some conversion for intuitive keyboard pan/tilt
                raw_delta = event_listener.HOT_KEY[keys['orn'][0]]
                delta = math_util.vec((raw_delta[1], -raw_delta[0])) * self._sens * 20

                signal['camera'].append(('orn', delta))
        else:
            # Handle environment settings
            for label, (key, status) in keys.items():
                # Using multiple if's to allow simultaneous key pressing
                if key in range(48, 58) and status == 'releasing':
                    # 'id' key_type
                    signal['tid'] = int(key) - 48
                if label == 'tool' and status == 'releasing':
                    signal['tid'] += event_listener.HOT_KEY[key]
                if label == 'key' and status == 'releasing':
                    signal[label] = event_listener.HOT_KEY[key]
                if label == 'rst' and status == 'pressing':
                    ins.append((label, 1))
                if label == 'grasp' and status == 'releasing':
                    ins.append((label, -1))
                if label == 'pos' and status == 'holding':
                    ins.append(('reach', (event_listener.HOT_KEY[key] * self._sens, None)))
                if label == 'orn' and status == 'holding':
                    orn = event_listener.HOT_KEY[key] * self._sens
                    # Don't touch position, only orientation (rad)
                    ins.append(('reach', (None, orn)))

        signal['update'] = 1 if 'tbd' in keys and keys['tbd'][1] == 'holding' else 0
        signal['instruction'] = ins
        # return signal

        if queue.full():
            # Force poll, then set back in
            queue.get_nowait()
        
        queue.put_nowait(signal)


# TODO at some point
class ViveEventHandler(ControlHandler):
    """
    Handles VR controller events/signal
    """
    def __init__(self, ps_id, queue, sensitivity=1, rate=100, qsize=200):
        super(ViveEventHandler, self).__init__(ps_id, queue, sensitivity, rate, qsize)

        # Initialize positions
        self._controllers = dict()
        self._orn_state = math_util.zero_vec(3)
        self._listener = event_listener.HTCVive()
        self._devices = self._listener.get_registered_device()

    @property
    def name(self):
        return 'VRControl'

    def interrupt(self, queue):

        signal = dict()
        signal['cmd'] = list()
        signal['camera'] = list()
        signal['tid'] = 0
        signal['key'] = 'm'
        signal['update'] = 0
        ins = list()

        if self._devices['controller']:
            events = self._listener.get_controller_state(self._devices['controller'][0])
            pose = self._listener.get_device_pose(self._devices['controller'][0])
            if not events or not pose:
                self._devices = self._listener.get_registered_device()
            else:

                slide = events['trigger']

                reset_flag = event_listener.KEY_STATUS[events['menu']]
                engage_flag = event_listener.KEY_STATUS[events['pad']]

                pos, orn = pose

        # self._listener.vibrate(3)

        # print(self._listener.get_device_pose(3))

        # for c_id, pos, orn, slide, _, _, button, _ in events:

                # engage_flag = event_listener.KEY_STATUS[button[32]]
                # reset_flag = event_listener.KEY_STATUS[button[1]]
                # scroll_flag = event_listener.KEY_STATUS[button[2]]

                # Always use the gripper slider
                ins.append(('grasp', 1))
                print(engage_flag)
                # Reset button
                if reset_flag == 'releasing':
                    ins.append(('rst', 1))

                # Engage button
                if engage_flag == 'pressing':
                    self._orn_state = math_util.quat2euler(orn)

                if engage_flag == 'holding':

                    r_orn = math_util.sign(
                        math_util.quat2euler(orn)
                        - self._orn_state, 1e-2) * 0.001
                    ins.append(('reach', (pos, r_orn * self._sens)))


        signal['instruction'] = ins

        if queue.full():
            # Force poll, then set back in
            queue.get_nowait()

        queue.put_nowait(signal)

    def _register(self):
        """
        Register the newly connected controller device
        :return: None
        """
        # TODO register devices dynamically
        pass



    def stop(self):
        self._listener.close()
        super(ViveEventHandler, self).stop()


class AppEventHandler(ControlHandler):
    """
    Handler for keyboard events/signal
    """

    def __init__(self, ps_id, queue, sensitivity=1, 
                 rate=100, qsize=200, channel_name='ios_channel'):
        super(AppEventHandler, self).__init__(ps_id, queue, sensitivity, rate, qsize)
        self._comm = network.RedisComm('localhost', port=6379, db=0)
        self._channel_name = channel_name
        self._comm.connect_to_channel(channel_name)

    @property
    def name(self):
        return 'PhoneControl'

    def interrupt(self, queue):

        signal = {}
        signal['cmd'] = list()
        signal['update'] = 0
        ins = list()
        signal['camera'] = list()
        signal['record'] = True

        events = event_listener.listen_to_redis(
            self._comm.channels[self._channel_name])
        # time.sleep(1. / self._rate)

        for event_dic in events:
            # TODO: key and id
            ins.append(('rst', event_dic['rst']))
            ins.append(('grasp', event_dic['grasp']))

            pos_delta = math_util.vec(event_dic['pos']) * self._sens
            orn_delta = math_util.vec(event_dic['orn']) * self._sens
            
            if not event_dic['camera']:
                ins.append(('reach', (pos_delta, None)))
                ins.append(('reach', (None, orn_delta)))
            else:
                signal['camera'].append(('pos', pos_delta * self._sens))
                orn = math_util.vec((orn_delta[1], -orn_delta[2], 0)) * self._sens * 20
                signal['camera'].append(('orn', orn))

        signal['instruction'] = ins

        if queue.full():
            # Force poll, then set back in
            queue.get_nowait()

        queue.put_nowait(signal)

    def stop(self):
        self._comm.disconnect()
        super(AppEventHandler, self).stop()
