
__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class Checker(object):
    """
    Check if given task is finished
    """
    def __init__(self, env_name):
        self._name = env_name

    @property
    def name(self):
        return 'TaskCompletionChecker'

    def custom_setup(self, world):

        """
        Customize world for some fine tunings that
        cannot be specified in env xml file
        :param world: the world object to be setup
        :return: None
        """
        if self._name == 'push_sawyer' or self._name == 'push_kuka':
            cube_pos = world.body['cube_0'].pos
            robot = world.body['titan_0']
            # Move the gripper towards the cube initially
            initial_gripper_pos = \
                (cube_pos[0] - 0.05, cube_pos[1], cube_pos[2] + 0.025)

            robot.tool_pos = (initial_gripper_pos, 200)

    def check(self, body_dict):

        if self._name == 'push_sawyer':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            if cube.pos[2] >= 0.69:
                # If the cube jumps too high
                return True, False

            # Consider the case when the cube is down on the ground
            if cube.pos[2] <= 0.06:
                done = True
                success = False

                # Only allow pushing towards one side, 
                # falling into one specific region
                if table.pos[1] - .275 <= cube.pos[1] <= table.pos[1] + .275 and \
                   table.pos[0] + .25 <= cube.pos[0] <= table.pos[0] + .65:
                    success = True

                return done, success

        elif self._name == 'push_kuka':

            cube = body_dict['cube_0']
            table = body_dict['table_0']

            if cube.pos[2] >= 0.69:
                # If the cube jumps too high
                return True, False
                
            # Consider the case when the cube is down on the ground
            if cube.pos[2] <= 0.06:
                done = True
                success = False

                # Only allow pushing towards one side, 
                # falling into one specific region
                if table.pos[1] - .275 <= cube.pos[1] <= table.pos[1] + .275 and \
                   table.pos[0] + .25 <= cube.pos[0] <= table.pos[0] + .65:
                    success = True

                return done, success

        return False, False

    def stop(self):
        return
