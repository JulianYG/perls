from .gripper import PrismaticGripper


class PR2Gripper(PrismaticGripper):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=None,
                 orn=None):
        path = path or 'pr2_gripper.urdf'
        pos = (0., 0., 0.7) if pos is None else pos
        orn = (0., 0., 0., 1.) if orn is None else orn
        super(PR2Gripper, self).__init__(tool_id, engine, path, pos, orn, 1, 3)

    def reset(self):
        self.joint_states = ([0, 2],
                             [self.joint_specs['upper'][0],
                              self.joint_specs['upper'][2]],
                             'position', {})
        PrismaticGripper.reset(self)

    def grasp(self, slide=-1):
        if slide != -1:
            slide = float(slide)
            self.joint_states = ([0, 2],
                                 [self.joint_specs['upper'][0] * (1. - slide),
                                  self.joint_specs['upper'][2] * (1. - slide)],
                                 'position', {})
            if slide == 0:
                self._close_grip = False
            elif slide == 1:
                self._close_grip = True
        else:
            if self._close_grip:
                # Release in this case
                self.joint_states = ([0, 2],
                                     [self.joint_specs['upper'][0],
                                      self.joint_specs['upper'][2]],
                                     'position', {})
                self._close_grip = False
            else:
                # Close in this case
                self.joint_states = ([0, 2],
                                     [self.joint_specs['lower'][0],
                                      self.joint_specs['lower'][2]],
                                     'position', {})
                self._close_grip = True

