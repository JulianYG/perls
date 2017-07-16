#!/usr/bin/env python

import os
import os.path as osp
import pybullet as p
import time

import numpy as np

from .base import GraphicsEngine
from ..utils import math_util, util
from ..utils.io_util import FONT, loginfo, parse_log

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class BulletRenderEngine(GraphicsEngine):
    """
    Rendering render from bullet physics
    """

    _FRAME_TYPES = dict(off=0, gui=1, cmd=2, vr=3, udp=4, tcp=5)

    _DISP_CONF = dict(gui_panel=1,
                      shadow=2,
                      wire_frame=3,
                      vr_teleporting=4,
                      vr_picking=5,
                      vr_render_controllers=6,
                      rendering=7,
                      # 8: Wait for pybullet
                      keyboard_shortcut=9,
                      mouse_picking=10)

    def __init__(self, disp_info,
                 job='run', video=False,
                 log_dir=''):
        """

        :param disp_info:

        the simulation frame, choose string
        among 'vr, gui, cmd (non-gui), upd, or tcp'.
        :param job: specify the run, choose string among
        'run, record, and replay'
        :param video:
        :param log_dir:
        """

        self._disp_name, self._frame, self._disp_args = \
            disp_info

        self._job = job

        self._record_video = video
        self._record_name = 'perls_record'

        self._logging_id = list()
        self._server_id = -1

        # Initialize logging paths
        log_dir = log_dir or \
                  osp.abspath(osp.join(osp.dirname(__file__), '../../log'))
        self._log_path = dict(
            device=osp.join(log_dir, 'device'),
            trajectory=osp.join(log_dir, 'trajectory'),
            video=osp.join(log_dir, 'video')
        )
        # If not exist, create the paths
        for path in self._log_path.values():
            if not osp.exists(path):
                os.makedirs(path)

        self._build()

    @property
    def ps_id(self):
        return self._server_id

    @property
    def info(self):
        """
        Get the basic info description of the world.
        :return: A dictionary of information of this
        world.
        {version/name, status, real time, id,
        step size (if async), running job, camera info,
        log info, record file name, max running time}
        """
        info_dic = dict(
            name='Bullet Graphics Engine: {}'.
                 format(self._disp_name),
            job=self._job,
            record_name=self._record_name,
            log_info=self._log_path,
            camera_info=self.camera
        )
        return info_dic

    @property
    def camera(self):
        if self._FRAME_TYPES[self._frame] == 1:
            info = p.getDebugVisualizerCamera(self._server_id)
            return dict(
                frame_width=info[0], frame_height=info[1],
                view_mat=np.array(info[2], dtype=np.float32).reshape((4, 4)).T,
                projection_mat=np.array(info[3], dtype=np.float32).reshape((4, 4)).T,
                up=np.where(info[4])[0][0],
                yaw=info[8], pitch=info[9], focal_len=info[10],
                focus=info[11],
            )
        return 'Camera not available under frame {}'.format(self._frame)

    @property
    def record_name(self):
        """
        Get the base name string of record file, either
        to save to or load from
        :return: String
        """
        return self._record_name

    @camera.setter
    def camera(self, params):
        if self._frame == 'gui':
            p.resetDebugVisualizerCamera(
                params['flen'],
                params['yaw'],
                params['pitch'],
                params['focus'],
                self._server_id
            )
        elif self._frame == 'vr':
            cam_pos, cam_orn = params
            p.setVRCameraState(
                rootPosition=cam_pos,
                rootOrientation=cam_orn,
                physicsClientId=self._server_id
            )
        else:
            loginfo('Cannot set camera under frame type <{}>.'.
                    format(self._frame),
                    FONT.warning)

    @record_name.setter
    def record_name(self, name):
        """

        :param name:
        :return:
        """
        # If name is empty string, use default name still
        self._record_name = name or self._record_name

    ###
    #  Helper functions
    def get_camera_pose(self, up=(0.,1.,0.), otype='quat'):
        view_matrix = self.camera['view_mat'].T
        perm = [2, 0, 1]
        if up == (0, 1, 0):
            view_matrix = view_matrix.dot(np.array(
                [[-1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]],
                dtype=np.float32
            ))

        transformation_matrix = math_util.mat_inv(view_matrix)
        pos = transformation_matrix[3, :3]
        orn = math_util.mat2euler(transformation_matrix[:3, :3],
                                  axes='sxyx')[perm]
        if otype == 'quat':
            orn = math_util.euler2quat(tuple(orn))
        elif otype == 'deg':
            orn = math_util.deg(orn)
        elif otype == 'rad':
            pass
        else:
            loginfo('Unrecognized orientation type. '
                    'Choose among quat, deg, and rad',
                    FONT.ignore)
        return pos, orn

    def _build(self):
        # Convert to bullet constant
        self._disp_args[0] = self._FRAME_TYPES[self._frame]
        # The core step: connect to bullet physics server
        self._server_id = p.connect(*self._disp_args)
        p.setInternalSimFlags(0, self._server_id)
        p.resetSimulation(self._server_id)

        # logerr('Incompatible frame type <{}>.'.
        #        format(self._frame), FONT.disp)

    ###
    # General display related methods

    def configure_display(self, config, camera_args):
        # All about cameras

        for name, switch in config.items():
            p.configureDebugVisualizer(
                self._DISP_CONF[name],
                switch, physicsClientId=self._server_id)

            # Setup camera
            self.camera = camera_args

    def disable_hotkeys(self):
        p.configureDebugVisualizer(9, 0, self._server_id)

    def boot(self, target_uids):
        if self._job == 'record':
            assert self._record_name, \
                'Must provide record file name!'
            time_stamp = util.get_full_time_stamp()

            # Record only objects that are tracked
            if target_uids:
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_GENERIC_ROBOT,
                        osp.join(self._log_path['trajectory'],
                                 '{}_{}.bin'.format(
                                     self._record_name, time_stamp)),
                        objectUniqueIds=target_uids,
                        physicsClientId=self._server_id
                    )
                )
            else:
                # Record every object
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_GENERIC_ROBOT,
                        osp.join(self._log_path['trajectory'],
                                 '{}_{}.bin'.format(
                                     self._record_name, time_stamp)),
                        physicsClientId=self._server_id
                    )
                )

            # Record mp4 video if indicated
            if self._record_video:
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_VIDEO_MP4,
                        osp.join(self._log_path['video'],
                                 '{}_{}.mp4'.format(
                                     self._record_name, time_stamp)),
                        physicsClientId=self._server_id
                    )
                )

            if self._frame == 'vr':
                # Record the egocentric camera pose
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_VR_CONTROLLERS,
                        osp.join(self._log_path['device'],
                                 '{}_{}.bin'.format(
                                     self._record_name, time_stamp)),
                        deviceTypeFilter=p.VR_DEVICE_HMD,
                        physicsClientId = self._server_id
                    )
                )
            return 2

        elif self._job == 'replay':

            objects = osp.join(self._log_path['trajectory'],
                               '{}.bin'.format(self._record_name))
            device = osp.join(self._log_path['device'],
                              '{}.bin'.format(self._record_name))

            # Can change verbosity later
            obj_log = parse_log(objects, verbose=False)

            # TODO: set camera angle for GUI/HMD
            loginfo('Start replaying file {}'.
                    format(self._record_name), FONT.control)
            for record in obj_log:
                # time_stamp = float(record[1])
                obj = record[2]
                pos = record[3: 6]
                orn = record[6: 10]
                p.resetBasePositionAndOrientation(obj, pos, orn)
                n_joints = p.getNumJoints(obj)
                for i in range(n_joints):
                    joint_info = p.getJointInfo(obj, i)
                    qid = joint_info[3]
                    if qid > -1:
                        p.resetJointState(obj, i, record[qid - 7 + 17])

                # TODO: think about changing delay
                time.sleep(1e-4)

            # TODO: figure out using HMD log to revive FPS
            return 1
        return 0

    def stop(self):

        # Stop state logging if any
        for lid in self._logging_id:
            p.stopStateLogging(lid, self._server_id)
