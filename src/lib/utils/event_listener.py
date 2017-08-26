import ast
from platform import system
from ctypes import sizeof, c_void_p

if sizeof(c_void_p) != 8 or system() != 'Darwin':
    import openvr

from pybullet import (getKeyboardEvents,
                      getVREvents,
                      getMouseEvents)
from .math_util import vec, mat4, mat2pose
from .io_util import loginfo, logerr, FONT

_X_POS_VEC = vec((.001, .0, .0))
_X_NEG_VEC = vec((-.001, .0, .0))

_Y_POS_VEC = vec((.0, .001, .0))
_Y_NEG_VEC = vec((.0, -.001, .0))

_Z_POS_VEC = vec((.0, .0, .001))
_Z_NEG_VEC = vec((.0, .0, -.001))

DEVICE_TYPE = dict(controller=2,
                   hmd=1,
                   generic=3,
                   monitor=4)

VR_STATUS = dict(
    application=0,  # openvr.VRApplication_Other,
    scene=1,   # openvr.VRApplication_Scene,
    overlay=2,  # openvr.VRApplication_Overlay,
    background=3,   # openvr.VRApplication_Background,
    util=4, # openvr.VRApplication_Utility,
    monitor=5,  # openvr.VRApplication_VRMonitor,
    watchdog=6, # openvr.VRApplication_SteamWatchdog,
    # bootstrap=7,    # openvr.VRApplication_Bootstrapper,
    max=7   # openvr.VRApplication_Max,
)

KEY_STATUS = {0: 'idle',
              1: 'holding',
              2: 'triggered',
              3: 'pressing',
              4: 'releasing',
              5: 'error',
              6: 'pre-pressed',
              7: 'invalid'
              }

KEY_LABEL = {65284: 'rst', # F5 for reset
             65306: 'cam', # for camera control, SHIFT
             65307: 'tbd', # reserved, CONTROL

             # 6dof Orientation control
             65295: 'orn',  # 'roll_counterclockwise', <--
             65296: 'orn',  # 'roll_clockwise', -->
             65297: 'orn',  # 'pitch_up', ^
             65298: 'orn',  # 'pitch_down', v
             114: 'orn',  # 'yaw_left', R
             102: 'orn',  # 'yaw_right, F

             # 6dof Position control
             119: 'pos',  # 'x_forward', W
             115: 'pos',  # 'x_backward', S
             97: 'pos',   # 'y_left', A
             100: 'pos',   # 'y_right', D
             101: 'pos',   # 'z_up', E
             113: 'pos',   # 'z_down', Q

             # Tool mega-control
             32: 'grasp',  # Space
             103: 'key',   # 'gripper', G
             109: 'key',   # 'arm', M
             44: 'tool',   # 'last_tool', ','
             46: 'tool',   # 'next_tool', '.'
             # 48 - 57 corresponds to 'id' type, 0-9
             }

HOT_KEY = {65284: None,  # F5
           65306: None,  # SHIFT
           65307: None,  # CONTROL

           # Tool Orientation control
           65295: _X_NEG_VEC,  # <^
           65296: _X_POS_VEC,  # ^>
           65297: _Y_POS_VEC,  # ^
           65298: _Y_NEG_VEC,  # v
           113: _Z_NEG_VEC,  # <--
           101: _Z_POS_VEC,  # -->

           # Tool Position control
           32: None,  # Space
           119: _X_NEG_VEC,  # W
           115: _X_POS_VEC,  # S
           97: _Y_NEG_VEC,  # A
           100: _Y_POS_VEC,  # D
           114: _Z_POS_VEC,  # R
           102: _Z_NEG_VEC,  # F
           103: 'g',
           109: 'm',
           44: -1,  # ,
           46: 1,  # .
           # 48 - 57 corresponds to their own values - 48.
           }


def listen_to_bullet_keyboard(ps_id=0):
    return {101: 3}#getKeyboardEvents(physicsClientId=ps_id)


def listen_to_bullet_mouse(ps_id=0):
    return getMouseEvents(physicsClientId=ps_id)


class ViveListener(object):

    def __init__(self, v_type='background'):

        self._vr_system = openvr.init(VR_STATUS[v_type])
        self._vr_compositor = openvr.IVRCompositor()

        self._devices = self.get_registered_device()

    def get_registered_device(self):

        devices = dict(hmd=[], controller=[],
                       generic=[], monitor=[])
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            d_type = self._vr_system.getTrackedDeviceClass(i)
            if d_type > 0:
                devices[DEVICE_TYPE[d_type]].append(i)

        return devices

    def get_controller_state(self, c_id):

        events = dict(menu=0,
                      grip=0,
                      pad=0,
                      pad_point=(0., 0.),
                      trigger=0.)

        if c_id not in self._devices['controller']:
            logerr('Given id {} does not correspond to controller.'.format(c_id),
                    FONT.warning)
            return

        # Get VR Events
        event_struct = openvr.VREvent_t()
        # Get controller states
        valid, x = self._vr_system.getControllerState(
            c_id, sizeof(openvr.VRControllerState_t))

        if valid:
            events['pad_point'] = (x.rAxis[0].x, x.rAxis[0].y)  # in range [-1, 1]
            events['trigger'] = x.rAxis[1].x  # in range [0, 1], 0 is unpressed, 1 is pressed

        if self._vr_system.pollNextEvent(event_struct):
            info = event_struct.data
            buttonID = info.controller.button  # identifies button

            if event_struct.trackedDeviceIndex == c_id:

                if buttonID == openvr.k_EButton_ApplicationMenu:

                    if event_struct.eventType == openvr.VREvent_ButtonPress:
                        events['menu'] = 2
                    elif event_struct.eventType == openvr.VREvent_ButtonUnpress:
                        events['menu'] = 4

                elif buttonID == openvr.k_EButton_Grip:

                    if event_struct.eventType == openvr.VREvent_ButtonPress:
                        events['grip'] = 2
                    elif event_struct.eventType == openvr.VREvent_ButtonUnpress:
                        events['grip'] = 4

                elif buttonID == openvr.k_EButton_SteamVR_Touchpad:

                    if event_struct.eventType == openvr.VREvent_ButtonPress:
                        events['pad'] = 2
                    elif event_struct.eventType == openvr.VREvent_ButtonUnpress:
                        events['pad'] = 4
        return events

    def get_device_pose(self, c_id):

        poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            openvr.k_unMaxTrackedDeviceCount)

        device_pose = poses[c_id]

        if device_pose.bDeviceIsConnected:

            pose = poses[5]
            self._vr_compositor.waitGetPoses(
                poses, openvr.k_unMaxTrackedDeviceCount, None, 0)

            pos, orn = mat2pose(
                mat4([
                    list(pose.mDeviceToAbsoluteTracking[0]),
                    list(pose.mDeviceToAbsoluteTracking[1]),
                    list(pose.mDeviceToAbsoluteTracking[2]),
                    [0, 0, 0, 1]
                ])
            )
            return tuple(pos[[0, 2, 1]]), tuple(orn)


def listen_to_redis(queue):
    """
    Listen to a connected and subscribed redis queue
    :param queue: the registered queue that
    receives data from callback function
    :return:
    """
    # TODO: construct socket class
    events = list()
    while not queue.empty():
        item = queue.get()
        # parses into AST (Source Tree),
        # only evaluates/returns if it's literal,
        # so no safety issue
        signal_dic = ast.literal_eval(item)
        events.append(signal_dic)
    return events

