from .gripper import PrismaticGripper


class RethinkGripper(PrismaticGripper):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=(0., 0., 0.5),
                 orn=(0., 0., 0., 1)):
        path = path or 'rethink_ee_description/urdf/' \
                       'electric_gripper/right_standard_narrow_round.urdf'
        PrismaticGripper.__init__(self, tool_id, engine, path, pos, orn, 1, 3)

    def reset(self):
        self.joint_states = ([1, 3],
                             [self.joint_specs['upper'][1],
                              self.joint_specs['lower'][3]],
                             'position', {})
        PrismaticGripper.reset(self)

    def grasp(self, slide=-1):
        if slide > -1:
            slide = float(slide)
            self.joint_states = ([1, 3],
                                 [self.joint_specs['upper'][1] * (1. - slide),
                                  self.joint_specs['lower'][3] * (1. - slide)],
                                 'position', {})
            if slide == 0.:
                self._close_grip = False
            elif slide == 1.:
                self._close_grip = True
        else:
            if self._close_grip:
                # Release in this case
                self.joint_states = ([1, 3],
                                     [self.joint_specs['upper'][1],
                                      self.joint_specs['lower'][3]],
                                     'position', {})
                self._close_grip = False
            else:
                # Close in this case
                self.joint_states = ([1, 3],
                                     [self.joint_specs['lower'][1],
                                      self.joint_specs['upper'][3]],
                                     'position', {})
                self._close_grip = True
