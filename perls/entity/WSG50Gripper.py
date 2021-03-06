from .gripper import PrismaticGripper
from ..utils import math_util


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

        self._tip_offset = math_util.vec((0., 0, 0.15))

    def grasp(self, slide=-1):
        if slide > -1:
            slide = float(slide)
            # TODO: figure out why the finger slides
            self.joint_positions = [
                None, self.joint_specs['upper'][1] +
                      slide * (self.joint_specs['upper'][1] -
                               self.joint_specs['lower'][1]),
                0, 0, self.joint_specs['upper'][4] * slide, None,
                self.joint_specs['upper'][6] * slide, None
            ]
            if slide == 0.:
                self._close_grip = False
            elif slide == 1.:
                self._close_grip = True
        else:
            if self._close_grip:
                # Release in this case
                self.joint_positions = [
                    None, self.joint_specs['upper'][1],
                    -0.22, 0.22, self.joint_specs['lower'][4],
                    None, self.joint_specs['lower'][6], None
                ]
                self._close_grip = False
            else:
                # Close in this case
                self.joint_positions = [
                    None, self.joint_specs['lower'][1],
                    -0.22, 0.22, self.joint_specs['upper'][4],
                    None, self.joint_specs['upper'][6], None
                ]
                self._close_grip = True
