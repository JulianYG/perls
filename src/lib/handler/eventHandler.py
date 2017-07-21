from .base import InterruptHandler
from ..utils import event_listener, math_util


class ViewEventHandler(InterruptHandler):

    def __init__(self, ps_id, ):

        super(ViewEventHandler, self).__init__(ps_id, None)
        self._signal = dict()
        self._angle = math_util.zero_vec(3)

    @property
    def name(self):
        return 'ViewEventHandler'

    @property
    def signal(self):
        events = event_listener.listen_to_bullet_keyboard(self._id)

        # Construct dictionary {label: (key, status)}
        keys = {event_listener.KEY_LABEL.get(int(long_key), None):
                    (int(long_key), event_listener.KEY_STATUS[int(const)])
                for (long_key, const) in events.items()}

        # TODO: probably need to change this into set_camera_pose
        if 'cam' in keys and keys['cam'][1] == 'holding':
            if 'pos' in keys and keys['pos'][1] == 'holding':
                self._signal['focus'] += event_listener.HOT_KEY[keys['pos'][0]] * 50

            if 'orn' in keys and keys['orn'][1] == 'holding':
                self._angle += math_util.deg(event_listener.HOT_KEY[keys['orn'][0]])
                self._signal['roll'] = self._angle[0]
                self._signal['pitch'] = self._angle[1]
                self._signal['yaw'] = self._angle[2]

        return self._signal

    def update_states(self, params):
        """
        Initialize/update the camera states
        :param params: (key string, value) tuple of parameters:
        roll, pitch, yaw (all in degrees), flen, focus (vec3 floats)
        :return: None
        """
        for key, val in params.items():
            self._signal[key] = val
        self._angle = math_util.vec(
            (self._signal['roll'], self._signal['pitch'],
             self._signal['yaw']))

    def stop(self):
        return


class AssetHandler(InterruptHandler):

    def __init__(self, ps_id, rate=100):
        super(AssetHandler, self).__init__(ps_id, rate)

    def save_assets(self):

        pass

    def add_asset(self):

        pass


    def create_asset(self):

        pass

    def click_to_pos(self):

        pass
    
    def parameter_tuning(self):
        
        pass

