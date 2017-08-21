from .base import InterruptHandler

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class Checker(InterruptHandler):
    """
    Check if given task is finished
    """
    def __init__(self, ps_id, env_name):
        super(Checker, self).__init__(ps_id, 100)
        self._name = env_name

    def check(self, body_dict):

        if self._name == 'push':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            if cube.pos[2] >= 0.68:
                # If the cube jumps too high
                return True, False

            # Consider the case when the cube is down on the ground
            if cube.pos[2] <= 0.06:
                done = True
                success = False


                # Only allow pushing towards one side, 
                # falling into one specific region
                if table.pos[1] -.275 <= cube.pos[1] <= table.pos[1] + .275 and \
                   table.pos[0] + .25 <= cube.pos[0] <= table.pos[0] + .65:
                    success = True

                return done, success

        return False, False

