from pybullet import (getKeyboardEvents,
                      getVREvents,
                      getMouseEvents)
import pybullet as p
from math_util import vec

_X_POS_VEC = vec((.001, .0, .0))
_X_NEG_VEC = vec((-.001, .0, .0))

_Y_POS_VEC = vec((.0, .001, .0))
_Y_NEG_VEC = vec((.0, -.001, .0))

_Z_POS_VEC = vec((.0, .0, .001))
_Z_NEG_VEC = vec((.0, .0, -.001))

DEVICE_TYPE = dict(controller=1,
                   head_mount=2,
                   generic=4)

KEY_STATUS = {1: 'pressed', 2: 'triggered', 4: 'released'}

KEY_LABEL = {65284: 'rst', # F5 for reset

             # Orientation control
             65295: 'orn',  # 'roll_counterclockwise', <--
             65296: 'orn',  # 'roll_clockwise', -->
             65297: 'orn',  # 'yaw_up', ^
             65298: 'orn',  # 'yaw_down', v

             # Position control
             119: 'pos',  # 'x_forward', W
             115: 'pos',  # 'x_backward', S
             97: 'pos',   # 'y_left', A
             100: 'pos',   # 'y_right', D
             114: 'pos',   # 'z_up', R
             102: 'pos',   # 'z_down', F

             # Tool mega-control
             32: 'grasp',  # Space
             103: 'key',   # 'gripper', G
             109: 'key',   # 'arm', M
             44: 'tool',   # 'last_tool', ','
             46: 'tool',   # 'next_tool', '.'
             # 48 - 57 corresponds to 'id' type, 0-9
             }

HOT_KEY = {65284: None, # F5
           # Orientation control
           65295: _X_NEG_VEC,  # <--
           65296: _X_POS_VEC,  # -->
           65297: _Y_POS_VEC,  # ^
           65298: _Y_NEG_VEC,  # v
           # Position control
           32: None,  # Space
           119: _X_POS_VEC,  # W
           115: _X_NEG_VEC,  # S
           97: _Y_NEG_VEC,  # A
           100: _Y_POS_VEC,  # D
           114: _Z_POS_VEC,  # R
           102: _Z_NEG_VEC,  # F
           103: 'g',
           104: 'm',
           44: -1,  # ,
           46: 1,  # .
           # 48 - 57 corresponds to their own values - 48.
          }


# TODO: All use id as 0 for now, assuming one server
# for keyboard control (GUI)


def listen_to_keyboard(id=0):
    return getKeyboardEvents(physicsClientId=id)


def listen_to_mouse(id=0):
    return getMouseEvents(physicsClientId=id)


def listen_to_vive(dtype, id=0):
    t = 0
    for device in (dtype,):
        t |= DEVICE_TYPE[device]
    return getVREvents(deviceTypeFilter=t,
                       physicsClientId=id)

