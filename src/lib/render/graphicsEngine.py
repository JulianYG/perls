#!/usr/bin/env python

import os
import os.path as osp
import pybullet as p

import numpy as np
import math

from ..utils import (io_util,
                     math_util, 
                     time_util, 
                     plot_util)
from ..utils.io_util import FONT, loginfo, parse_log

from .renderEngine import GraphicsEngine

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class OpenGLEngine(GraphicsEngine):

    pass


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
        self._replay_delay = 1e-4

        self._logging_id = list()
        self._server_id = -1

        # Initialize logging paths
        log_dir = log_dir or \
                  osp.abspath(osp.join(osp.dirname(__file__), '../../log'))
        self._log_path = dict(
            device=osp.join(log_dir, 'device'),
            trajectory=osp.join(log_dir, 'trajectory'),
            success_dir=osp.join(log_dir, 'trajectory', 'success'),
            fail_dir=osp.join(log_dir, 'trajectory', 'fail'),
            video=osp.join(log_dir, 'video')
        )

        self._base_file_name = ''

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
    def frame(self):
        return self._frame

    @property
    def camera(self):
        if self._FRAME_TYPES[self._frame] == 1 or \
           self._FRAME_TYPES[self._frame] == 3:
            info = p.getDebugVisualizerCamera(self._server_id)
            return dict(
                frame_width=info[0], frame_height=info[1],
                view_mat=math_util.mat4(info[2]).T,
                projection_mat=math_util.mat4(info[3]).T,
                up=np.where(info[4])[0][0],
                forward=math_util.vec(info[5]),
                yaw=info[8], pitch=info[9], focal_len=info[10],
                focus=math_util.vec(info[11]),
            )
        else:
            loginfo('Camera not available under frame {}'.format(self._frame), FONT.ignore)
            return {}

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
        # elif self._frame == 'vr':
        #     cam_pos, cam_orn = params
        #     p.setVRCameraState(
        #         rootPosition=cam_pos,
        #         rootOrientation=cam_orn,
        #         physicsClientId=self._server_id
        #     )
        # else:
        #     loginfo('Cannot set camera under frame type <{}>.'.
        #             format(self._frame),
        #             FONT.warning)

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

        # If up axis is y
        if up == 1:
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
                                  axes='rxyx')

        # This is some weird bullet convention..
        # To match the degrees converted from transformation matrix into
        # pitch/yaw angles from bullet debug visualizer
        if math.sin(orn[2]) > 0:
            orn[0] = - (np.pi + orn[0])
        else:
            orn[0] = - orn[0]
            orn[1] = np.pi * 2 - orn[1]

        if otype == 'mat':
            orn = transformation_matrix[:3, :3]
        elif otype == 'quat':
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

    def set_camera_pose(self, pos, orn, up=(0., 1., 0.)):
        # TODO
        pass

    def _build(self):
        # Convert to bullet constant
        self._disp_args[0] = self._FRAME_TYPES[self._frame]
        # The core step: connect to bullet physics server
        if self._frame != 'vr' or self._job == 'replay':
            self._server_id = p.connect(*self._disp_args)
        else:
            self._server_id = p.connect(self._disp_args[0])
        p.setInternalSimFlags(0, self._server_id)
        p.resetSimulation(self._server_id)

        # logerr('Incompatible frame type <{}>.'.
        #        format(self._frame), FONT.disp)

    ###
    # General display related methods

    def configure_display(self, config, camera_args, replay_args):
        # All about cameras

        for name, switch in config.items():
            p.configureDebugVisualizer(
                self._DISP_CONF[name],
                switch, physicsClientId=self._server_id)

        # Setup camera
        self.camera = camera_args
        self._replay_delay = replay_args['delay']

    def disable_hotkeys(self):
        p.configureDebugVisualizer(9, 0, self._server_id)

    def get_camera_image(self, itype):

        camera_param = self.camera
        width, height, rgb_img, depth_img, seg_img = \
            p.getCameraImage(camera_param['frame_width'],
                             camera_param['frame_height'],
                             camera_param['view_mat'],
                             camera_param['projection_mat'],
                             [0, 1, 0], [1, 1, 1],
                             renderer=p.ER_TINY_RENDERER)
        if itype == 'human':
            rgb_img = np.reshape(rgb_img, (height, width, 4)).astype(np.float32) / 255.
            plot_util.pop(rgb_img, 1.5, dict(interpolation='none'))

        elif itype == 'rgb':
            return np.reshape(rgb_img, (height, width, 4)).astype(np.float32) / 255.

        elif itype == 'depth':
            return np.reshape(depth_img, (height, width)).astype(np.float32)

        elif itype == 'segment':
            return np.reshape(seg_img, (height, width)).astype(np.float32)

        else:
            loginfo('Unrecognized image type', FONT.ignore)

    def boot(self, target_uids):
        if self._job == 'record':
            assert self._record_name, \
                'Must provide record file name!'
            time_stamp = time_util.get_full_time_stamp()

            self._base_file_name = '{}_{}.bin'.format(
                self._record_name, time_stamp)

            abs_file_name = osp.join(
                self._log_path['trajectory'],
                self._base_file_name)

            # Record only objects that are tracked
            if target_uids:
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_GENERIC_ROBOT,
                        abs_file_name,
                        objectUniqueIds=target_uids,
                        physicsClientId=self._server_id
                    )
                )
            else:
                # Record every object
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_GENERIC_ROBOT,
                        abs_file_name,
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
                        abs_file_name,
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

                time_util.pause(self._replay_delay)

            loginfo('Finished replay {}'.format(self._record_name),
                    FONT.control)
            # TODO: figure out using HMD log to revive FPS
            return 1
        return 0

    def stop(self, exit_code):

        # Stop state logging if any
        for lid in self._logging_id:
            p.stopStateLogging(lid, self._server_id)

        if self._job == 'record':
            if exit_code < 0:
                loginfo('Record file not classified.', FONT.ignore)

            # Save success and failure cases separately
            elif exit_code == 0:
                io_util.fmove(
                    osp.join(
                        self._log_path['trajectory'],
                        self._base_file_name), 
                    osp.join(
                        self._log_path['success_dir'],
                        self._base_file_name)
                )

            # Just ignore the case of error
            elif exit_code == 1:
                io_util.fmove(
                    osp.join(
                        self._log_path['trajectory'],
                        self._base_file_name), 
                    osp.join(
                        self._log_path['fail_dir'],
                        self._base_file_name)
                )
