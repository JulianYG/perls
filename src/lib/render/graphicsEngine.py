#!/usr/bin/env python

import os, sys
import os.path as osp
import pybullet as p

import numpy as np
import math
import logging

from ..utils import (io_util,
                     math_util, 
                     time_util, 
                     plot_util)
from ..utils.io_util import parse_log, pjoin, PerlsLogger

from .renderEngine import GraphicsEngine

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

logging.setLoggerClass(PerlsLogger)


class OpenGLEngine(GraphicsEngine):

    pass


class BulletRenderEngine(GraphicsEngine):
    """
    Rendering render from bullet physics
    """

    FRAME_TYPES = dict(off=0, gui=1, cmd=2, vr=3, udp=4, tcp=5)

    JOB_TYPES = dict(run=0, record=1, replay=2)

    DISP_CONF = dict(gui_panel=1,
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
                 log_dir='', task=''):
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

        # Default settings for rendering
        self._render_param = dict(
            frame_width=224, frame_height=224,
            view_mat=p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=(0, 0, 0), 
                    distance=5, 
                    yaw=50, pitch=-35, roll=0, 
                    upAxisIndex=2),
            projection_mat=p.computeProjectionMatrixFOV(60, 1, 0.02, 100),
            up=2,
            forward=(-0.6275, 0.5265, -0.5736),
            yaw=50, pitch=-35, flen=5,
            focus=(0, 0, 0)
        )
        self._job = job
        self._record_files = []

        self._record_video = video
        self._record_name = task
        self._replay_delay = 1e-4

        self._logging_id = list()
        self._replay_count = 0
        self._server_id = -1
        self._active = False

        # Initialize logging paths
        log_dir = log_dir or \
                  osp.abspath(pjoin(osp.dirname(__file__), '../../log'))
        self._log_path = dict(
            root=log_dir,
            device=pjoin(log_dir, 'device'),
            trajectory=pjoin(log_dir, 'trajectory'),
            success_rl=pjoin(log_dir, 'learning',
                             self._record_name, 'success'),
            fail_rl=pjoin(log_dir, 'learning',
                          self._record_name, 'fail'),
            success_trajectory=pjoin(log_dir, 'trajectory',
                                     self._record_name, 'success'),
            fail_trajectory=pjoin(log_dir, 'trajectory',
                                  self._record_name, 'fail'),
            video=pjoin(log_dir, 'video')
        )

        self._base_file_name = self._record_name

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
            record_name=self._base_file_name,
            log_info=self._log_path,
            camera_info=self.camera,
            file_count=len(self._record_files),
        )
        return info_dic

    @property
    def frame(self):
        return self._frame

    @property
    def camera(self):
        if self._frame == 'vr' or self._frame == 'gui':
            info = p.getDebugVisualizerCamera(self._server_id)
            return dict(
                frame_width=info[0], frame_height=info[1],
                view_mat=info[2],
                projection_mat=info[3],
                up=np.where(info[4])[0][0],
                forward=math_util.vec(info[5]),
                yaw=info[8], pitch=info[9], flen=info[10],
                focus=math_util.vec(info[11]),
            )
        else:
            return self._render_param

    @property
    def record_dir(self):
        """
        Get the base name string of record file, either
        to save to or load from
        :return: String
        """
        return self._record_name

    @camera.setter
    def camera(self, params):
        if self._frame == 'gui':

            # Bullet has a bug at this step.
            # The same settings results in different
            # camera pose readings between CMD mode
            # and GUI mode, and CMD mode is correct.
            p.resetDebugVisualizerCamera(
                cameraDistance=params['flen'],
                cameraYaw=params['yaw'],
                cameraPitch=params['pitch'],
                cameraTargetPosition=params['focus'],
                physicsClientId=self._server_id
            )
        elif self._job == 'replay' and self._frame == 'vr':
            cam_pos, cam_orn = params
            p.setVRCameraState(
                rootPosition=cam_pos,
                rootOrientation=cam_orn,
                physicsClientId=self._server_id
            )
        # For non-visualization modes
        else:
            if 'dim' in params:
                self._render_param['frame_width'] = params['dim'][0]
                self._render_param['frame_height'] = params['dim'][1]

            for name, val in params.items():
                if name != 'dim':
                    self._render_param[name] = val

            # Update view matrix again
            self._render_param['view_mat'] = \
                p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=self._render_param['focus'], 
                    distance=self._render_param['flen'],
                    yaw=self._render_param['yaw'], 
                    pitch=self._render_param['pitch'], 
                    roll=0, 
                    upAxisIndex=2)

    @record_dir.setter
    def record_dir(self, name):
        """

        :param name:
        :return:
        """
        # If name is empty string, use default name still
        self._record_files = io_util.flist(
                pjoin(self._log_path['trajectory'], 
                      name) + '/*bin')

    ###
    #  Helper functions
    def get_camera_pose(self, otype='quat'):

        view_matrix = math_util.mat4(self.camera['view_mat'])

        # If up axis is y
        if self.camera['up'] == 1:
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

        # In [roll, pitch, yaw] form;
        # by definition roll is fixed to zero
        orn = (0, orn[0], orn[1])
        if otype == 'mat':
            orn = transformation_matrix[:3, :3]
        elif otype == 'quat':
            orn = math_util.euler2quat(orn)
        elif otype == 'deg':
            orn = math_util.deg(orn)
        elif otype == 'rad':
            pass
        else:
            logging.warning('Unrecognized orientation type. '
                    'Choose among quat, deg, and rad')
        return pos, orn        

    def set_camera_pose(self, pos, orn, upAxisIdx=1):
        # TODO
        pass

    def _build(self):

        # Convert to bullet constant
        self._disp_args[0] = self.FRAME_TYPES[self._frame]

        # The core step: connect to bullet physics server
        if self._job == 'replay':
            self._server_id = p.connect(p.GUI)
        elif self._frame != 'vr':
            self._server_id = p.connect(*self._disp_args)
        else:
            self._server_id = p.connect(self._disp_args[0])

        # For old version VR, clean up 
        p.setInternalSimFlags(0, self._server_id)
        p.resetSimulation(self._server_id)

        if self._frame != 'vr' and self._frame != 'gui':
            logging.info('Real camera not available under frame {}, '
                    'use virtual settings instead'.format(self._frame))

    ###
    # General display related methods

    def configure_display(self, config, camera_args, replay_args):
        
        # Stop rendering for faster loading
        p.configureDebugVisualizer(
            p.COV_ENABLE_RENDERING, 0, 
            self._server_id)

        # All about cameras
        for name, switch in config.items():
            p.configureDebugVisualizer(
                self.DISP_CONF[name],
                switch, physicsClientId=self._server_id)

        # Setup camera
        self.camera = camera_args
        self._replay_delay = replay_args['delay']

    def disable_hotkeys(self):
        """
        Shutdown bullet/openGL built-in hotkeys for keyboard control
        """
        p.configureDebugVisualizer(9, 0, self._server_id)

    def get_camera_image(self, itype):

        camera_param = self.camera
        width, height, rgb_img, depth_img, seg_img = \
            p.getCameraImage(
                camera_param['frame_width'],
                camera_param['frame_height'],
                viewMatrix=camera_param['view_mat'],
                projectionMatrix=camera_param['projection_mat'],
                lightDirection=[0, 1, 0], 
                lightColor=[1, 1, 1],
                lightDistance=camera_param['flen'] + 1,
                shadow=0,
                # ... ambient diffuse, specular coeffs
                lightAmbientCoeff=.9,
                # Seems only able to use w/o openGL
                renderer=p.ER_TINY_RENDERER)

        if itype == 'human':
            rgb_img = np.reshape(rgb_img, (height, width, 4)).astype(np.float32) / 255.
            plot_util.pop(rgb_img[:, :, :3], 1.5, dict(interpolation='none'))

        elif itype == 'rgb':
            return np.reshape(rgb_img, (height, width, 4))[:, :, :3].astype(np.float32) / 255.

        elif itype == 'rgbd':
            rgbd = np.reshape(rgb_img, (height, width, 4)).astype(np.float32) / 255.
            # Now replace channel 3 (alpha) with depth
            rgbd[:, :, 3] = np.reshape(depth_img, (height, width)).astype(np.float32)
            return rgbd

        elif itype == 'depth':
            return np.reshape(depth_img, (height, width)).astype(np.float32)

        elif itype == 'segment':
            return np.reshape(seg_img, (height, width)).astype(np.float32)
        else:
            logging.warning('Unrecognized image type')

    def activate(self):
        if not self._active:
            p.configureDebugVisualizer(
                p.COV_ENABLE_RENDERING, 1, 
                self._server_id)
            self._active = True

    def boot(self, target_uids):

        if self._job == 'record':
            assert self._record_name, \
                'Must provide record file name!'
            time_stamp = time_util.get_full_time_stamp()

            self._base_file_name = '{}.bin'.format(time_stamp)

            abs_file_name = pjoin(
                self._log_path['trajectory'],
                self._base_file_name)

            # Record every object for visual replay purpose
            self._logging_id.append(
                p.startStateLogging(
                    p.STATE_LOGGING_GENERIC_ROBOT,
                    abs_file_name,
                    # Most commonly for 7 Dof robots
                    maxLogDof=7,
                    objectUniqueIds=target_uids,
                    logFlags=p.STATE_LOG_JOINT_MOTOR_TORQUES,
                    physicsClientId=self._server_id
                )
            )

            # Record mp4 video if indicated
            if self._record_video:
                self._logging_id.append(
                    p.startStateLogging(
                        p.STATE_LOGGING_VIDEO_MP4,
                        pjoin(self._log_path['video'],
                              self._record_name,
                              '{}.mp4'.format(time_stamp)),
                        physicsClientId=self._server_id
                    )
                )

            # TODO: may record under ViveListener
            # # Cannot record VR Device pose since running 
            # # independent openvr
            # if self._frame == 'vr':
            #     # Record the egocentric camera pose
            #     self._logging_id.append(
            #         p.startStateLogging(
            #             p.STATE_LOGGING_VR_CONTROLLERS,
            #             abs_file_name,
            #             deviceTypeFilter=p.VR_DEVICE_HMD,
            #             physicsClientId = self._server_id
            #         )
            #     )
            return 2

        elif self._job == 'replay':

            objects = pjoin(self._log_path['trajectory'],
                            self._record_files[self._replay_count])
            # device = pjoin(self._log_path['device'],
            #                '{}.bin'.format(self._record_name))

            file_name = osp.basename(objects)

            # Can change verbosity later
            obj_log = parse_log(objects, verbose=False)

            # TODO: set camera angle for GUI/HMD
            logging.info('Start replaying file {}'.
                    format(file_name))

            self.activate()

            try:
                for record in obj_log:
                    
                    ### Each record has following format:
                    # 'stepCount', 'timeStamp', 'objectId', 
                    # 'posX', 'posY', 'posZ', 
                    # 'oriX', 'oriY', 'oriZ', 'oriW', 
                    # 'velX', 'velY', 'velZ',
                    # 'omegaX', 'omegaY', 'omegaZ', 'nDOFs',
                    # 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                    # 'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6',
                    # 't0', 't1', 't2', 't3', 't4', 't5', 't6'
                    # where q, u, t stand for joint pos, vel, torq
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
            except KeyboardInterrupt:
                logging.info('Cancelled replaying file {}'.format(file_name))
                return 3

            logging.info('Finished replay {}'.format(file_name))
            self._replay_count += 1

            # TODO: figure out using HMD log to replicate first person view
            return 1
        return 0

    def stop(self, exit_code):

        # Stop state logging if any
        if self._logging_id:
            for lid in self._logging_id:
                p.stopStateLogging(lid, self._server_id)
            logging.info('Stop recording.')

        if self._job == 'record':

            # Just ignore the case of error or cancellation
            if exit_code < 0:
                logging.info('Record file discarded.')
                io_util.fdelete(
                    pjoin(
                        self._log_path['trajectory'],
                        self._base_file_name)
                )

            # Save for success cases
            elif exit_code == 0:
                io_util.fmove(
                    pjoin(
                        self._log_path['trajectory'],
                        self._base_file_name), 
                    pjoin(
                        self._log_path['success_trajectory'],
                        self._base_file_name)
                )

            # Save for failure cases
            elif exit_code > 0:
                io_util.fmove(
                    pjoin(
                        self._log_path['trajectory'],
                        self._base_file_name), 
                    pjoin(
                        self._log_path['fail_trajectory'],
                        self._base_file_name)
                )
