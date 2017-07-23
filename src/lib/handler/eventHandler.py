from .base import InterruptHandler
from ..utils import event_listener, math_util


class InteractiveHandler(InterruptHandler):

    def __init__(self, ps_id, ):

        super(InteractiveHandler, self).__init__(ps_id, None)


    @property
    def name(self):
        return 'InteractiveHandler'

    @property
    def signal(self):
        pass

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

