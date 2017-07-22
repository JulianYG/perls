from .base import InterruptHandler
from ..utils import event_listener, math_util


class ViewEventHandler(InterruptHandler):

    def __init__(self, ps_id, ):

        super(ViewEventHandler, self).__init__(ps_id, None)
        self._signal = dict(update=0, flen=1e-3, yaw=0, pitch=0)
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

                raw_vec = event_listener.HOT_KEY[keys['pos'][0]] * 30

                # TODO Align it with view perspective frame
                # transformed_vec = self._pose[:3, :3].T.dot(raw_vec)

                self._signal['flen'] = 1e-3
                self._signal['focus'] += raw_vec  # transformed_vec

            if 'orn' in keys and keys['orn'][1] == 'holding':

                # Some conversion for intuitive keyboard pan/tilt
                raw_delta = event_listener.HOT_KEY[keys['orn'][0]]
                delta = math_util.vec((raw_delta[2], raw_delta[1], -raw_delta[0]))

                self._angle += math_util.deg(delta) * 20
                self._signal['pitch'] = self._angle[1]
                self._signal['yaw'] = self._angle[2]

        self._signal['update'] = 1 if 'tbd' in keys and keys['tbd'][1] == 'holding' else 0
        return self._signal

    def update_states(self, pose):
        """
        Initialize/update the camera states
        :param pose: the (pos, orn) tuple of camera pose,
        where orn is in degrees of bullet convention
        :return: None
        """
        pos, orn = pose
        self._signal['focus'] = pos
        self._signal['yaw'] = orn[2]
        self._signal['pitch'] = orn[1]

        self._angle[1] = orn[1]
        self._angle[2] = orn[2]

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

