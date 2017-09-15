from .gripper import PrismaticGripper
from ..utils import math_util


class RethinkGripper(PrismaticGripper):

    def __init__(self, tool_id,
                 engine,
                 path=None,
                 pos=None,
                 orn=None):
        path = path or 'rethink_ee_description/urdf/' \
                       'electric_gripper/right_standard_narrow_round.urdf'
        pos = (0., 0., 0.7) if pos is None else pos
        # Exactly align with world frame
        orn = (0., 0., 0., 1.) if orn is None else orn
        super(RethinkGripper, self).__init__(tool_id, engine, path, pos, orn, 1, 3)
        self._tip_offset = math_util.vec((0, 0.0725, 0))

    def grasp(self, slide=-1):
        if slide > -1:
            slide = float(slide)
            self.joint_positions = [
                None, self.joint_specs['upper'][1] * (1. - slide),
                None, self.joint_specs['lower'][3] * (1. - slide), None
            ]
            if slide == 0.:
                self._close_grip = False
            elif slide == 1.:
                self._close_grip = True
        else:
            if self._close_grip:
                # Release in this case
                self.joint_positions = [
                    None, self.joint_specs['upper'][1], None,
                    self.joint_specs['lower'][3], None]
                self._close_grip = False
            else:
                # Close in this case
                self.joint_positions = [
                    None, self.joint_specs['lower'][1], None,
                    self.joint_specs['upper'][3],None]
                self._close_grip = True
