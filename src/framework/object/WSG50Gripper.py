from .gripper import PrismaticGripper


class WSG50Gripper(PrismaticGripper):

    def __init__(self,
                 tool_id, engine,
                 path=None,
                 pos=None,
                 orn=None):
        path = path or 'gripper/wsg50_one_motor_gripper_new_free_base.sdf'
        pos = (0., 0., 0.7) if pos is None else pos
        orn = (0., 0., 0., 1.) if orn is None else orn
        super(WSG50Gripper, self).__init__(tool_id, engine, path, pos, orn, 4, 6)

    def reset(self):
        self.joint_states = ([1, 2, 3, 4, 6],
                             [-0.01130, 0, 0.0,
                              self.joint_specs['lower'][4],
                              self.joint_specs['lower'][6]],
                             'position', {})

        PrismaticGripper.reset(self)

    def grasp(self, slide=-1):
        if slide > -1:
            slide = float(slide)
            self.joint_states = ([1, 2, 3, 4, 6],
                                 [-0.01130, -0.20, 0.20,
                                  self.joint_specs['upper'][4] * slide,
                                  self.joint_specs['upper'][6] * slide],
                                 'position', {})
            if slide == 0.:
                self._close_grip = False
            elif slide == 1.:
                self._close_grip = True
        else:
            if self._close_grip:
                # Release in this case
                self.joint_states = ([1, 2, 3, 4, 6],
                                 [-0.01130, -20, 20,
                                  self.joint_specs['lower'][4],
                                  self.joint_specs['lower'][6]],
                                 'position', {})
                self._close_grip = False
            else:
                # Close in this case
                self.joint_states = ([1, 2, 3, 4, 6],
                                 [-0.01130, 20, -20,
                                  self.joint_specs['upper'][4],
                                  self.joint_specs['upper'][6]],
                                 'position', {})
                self._close_grip = True
