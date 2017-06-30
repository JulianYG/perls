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

    def grasp(self, slide=-1):
        if slide > -1:
            slide = float(slide)
            self.joint_states = ([1, 2, 3, 4, 6],
                                 # [-0.011130, -0.206421,
								 # 0.205143, -0.009999, -0.010055],
                                 [0., 0, 0,
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
                                  #    [-0.011130, -0.206421,
								 # 0.205143, -0.009999, -0.010055],
                                 [0., -5, 5,
                                  self.joint_specs['lower'][4],
                                  self.joint_specs['lower'][6]],
                                 'position', {})
                self._close_grip = False
            else:
                # Close in this case
                self.joint_states = ([1, 2, 3, 4, 6],
                                     # [0.6855956234759611,
                                     #  -0.7479294372303137, 0.05054599996976922,
                                     #  0.049838105678835724],
                                 [0., 5, -5,
                                  self.joint_specs['upper'][4],
                                  self.joint_specs['upper'][6]],
                                 'position', {})
                self._close_grip = True
