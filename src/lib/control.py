import multiprocessing
import sys, logging
import platform

from .state import physicsEngine#, robotEngine
from .adapter import Adapter
from .render import graphicsEngine, camera
from .handler.base import NullHandler
from .handler.eventHandler import InteractiveHandler
from .handler.controlHandler import (KeyboardEventHandler,
                                     ViveEventHandler,
                                     AppEventHandler)
from .utils import io_util, time_util, math_util
from .utils.io_util import pjoin
from .utils.io_util import PerlsLogger
from .view import View
from .world import World

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

logging.setLoggerClass(PerlsLogger)

Logger = PerlsLogger('{}.log'.format(
    time_util.get_full_time_stamp()),
    use_color=False \
    if platform.system() == 'Windows' else True
)


class Controller(object):
    """
    The controller in MVC architecture.
    """
    #######
    # module type parsers

    _PHYSICS_ENGINES = dict(
        # Simulation
        bullet=physicsEngine.BulletPhysicsEngine,
        mujoco=physicsEngine.MujocoEngine,
        # gazebo=physicsEngine.OpenRaveEngine,

        # Reality
        # intera=robotEngine.InteraEngine,
    )

    _GRAPHICS_ENGINES = dict(
        # Simulation
        bullet=graphicsEngine.BulletRenderEngine,

        # Reality
        kinect=camera.Kinect,
    )

    _CTRL_HANDLERS = dict(
        keyboard=KeyboardEventHandler,
        vive=ViveEventHandler,
        phone=AppEventHandler,
        off=NullHandler
    )

    _LOGGING_LEVELS = dict(
        debug=logging.DEBUG,
        release=logging.WARNING,
        test=logging.INFO
    )

    def __init__(self, config_batch):
        """
        Initialize the controller
        :param config_batch: the file path of configuration,
        in xml format. It contains path to both env description
        and view description.
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Note: Currently only able to run in multiple physics
        servers without GUI, along with async simulation.
        Suggest to start with single server, real time GUI first.
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        The config should specify the same set of
        parameters for each physics server. Each physics
        server stands for one simulation running. This
        allows spawning/parallel running of multiple simulation
        for training purposes.
        """
        self._config = pjoin(io_util.pwd(__file__),
                             '../../configs',
                             config_batch)
        self._physics_servers = dict()
        self._process_pool = list()

        # space to store useful states.
        # Note it needs to use dictionary to store the states of
        # tools because the positions and orientations of the
        # tools cannot be determined before the control loop
        # has started for a few iterations, so that the
        # control handler cannot store the initial states of the
        # tools.
        self._states = \
            dict(tool=dict(),
                 log=list(),  # The log of commands or user
                 # operations/modifications on the world
                 camera=dict(flen=1e-3),  # The camera parameters
                 )
        self._num_of_runs = 1

        # Recording data in specific format needed
        self._init_time_stamp = None

        # For coonsistency across different clock speeds
        self._update_time_stamp = None
        self._build()

    @property
    def info(self):
        """
        Get information of current running controller
        :return: a dictionary of dictionaries:
        {instance_id:
            {designated job: run/record/replay,
             status: pending/stopped/killed/error/finished,
             world_name: task property related,
             view_name: exp property related,
             ... etc, all world/disp/physics_engine related info},
             control_type: keyboard/vr/...,
             running time: steps/ seconds
         instance_id: ......}
        """
        info_dic = {}
        for s_id, (world, disp, pe, ctrl_hdlr) in \
                self._physics_servers.items():
            info_dic[s_id] = dict(
                world_info=world.info,
                display_info=disp.info,
                engine_info=pe.info,
                control=ctrl_hdlr.name,
                run_time=time_util.get_elapsed_time(
                    self._init_time_stamp),
            )
        return info_dic

    def _build(self):
        """
        Private method to build the controller.
        :return: None
        """
        # A list of parsed configurations
        configs = io_util.parse_config(self._config)

        # If there are multiple configurations, we need to examine
        # if the rest are not GUI frame, and all should be async
        num_configs = len(configs)
        for i in range(num_configs):
            conf = configs[i]
            assert i == conf.id, \
                'Error loading configuration: invalid id.'

            # load configs
            queue = multiprocessing.Queue()
            nruns, world, disp, ctrl_hdlr = self.load_config(conf, queue)
            if num_configs > 1 and (not conf.async):
                err_control('Currently only support multiple instances '
                       'of non-GUI frame and asynchronous simulation. '
                       'GUI or synchronous frame can only run as '
                       'single simulation instance. \nSimulation '
                       'configuration %d build skipped with error. ' % i,
                       FONT.control)
                world.notify_engine('error')
            else:
                logging.info_control('Simulation configuration {} build success. '
                        'Build type: {}'.format(i, conf.build),
                        FONT.control)
                world.notify_engine('pending')
            self._physics_servers[conf.id] = (nruns, world, disp, ctrl_hdlr, queue)

    @staticmethod
    def load_config(conf, queue):
        """
        Helper method to load configurations. Not recommended with
        outside calls unless perform manual check, since it may
        conflict with async/frame settings and cause bad behaviors,
        such as fall into control loop and cannot enter the loop
        of the next server, or challenge the pybullet limitation
        of one local in-process GUI connection.
        Note conf.id must be natural numbers in increasing order
        starting from 0, as in [0, 1, 2, 3, ...]
        :param conf: configuration to load.
        Currently only support three types of engines:
        Bullet Physics, Mujoco, and Gazebo.
        :return: loaded tuple (world, display, physics physics_engine)
        """

        # Initialize graphics render (rendering render)
        ge = Controller._GRAPHICS_ENGINES[conf.graphics_engine](
            conf.disp_info,
            conf.job,
            conf.video,
            conf.log,
            conf.config_name
        )

        # Initialize physics render (state render)
        pe = Controller._PHYSICS_ENGINES[conf.physics_engine](
            conf.id,
            ge.ps_id,
            conf.max_run_time,
            conf.async,
            conf.step_size,
        )

        world = World(conf.model_desc, pe)
        display = View(conf.view_desc, Adapter(world), ge)

        # Set up control event interruption handlers
        ctrl_handler = Controller._CTRL_HANDLERS[conf.control_type](
            pe.ps_id, queue, conf.sensitivity, conf.rate,
        )

        # TODO
        Logger.setLevel(Controller._LOGGING_LEVELS[conf.build])

        num_of_runs = conf.num_of_runs

        # Give record name for physics physics_engine
        if conf.job == 'record':
            ge.record_dir = \
                conf.config_name or \
                '{}_{}'.format(world.info['name'],
                               display.info['name'])
        elif conf.job == 'replay':
            ge.record_dir = '{}/{}'.format(
                conf.config_name, conf.replay_name)

            # Cannot play more than number of files under directory
            num_of_runs = min(num_of_runs, ge.info['file_count'])

        # connect to bullet graphics/display render server,
        # Build display first to load world faster
        display.build()
        world.build()

        # Special case for keyboard control on View side
        if conf.control_type == 'keyboard':
            # Disable keyboard shortcuts for keyboard control
            display.disable_hotkeys()

        return num_of_runs, world, display, ctrl_handler

    def start_all(self):
        """
        Kick start all instances simulation!
        Note this can only be called when there are
        multiple instances of simulation servers.
        Each simulation will happen in different
        threads.
        :return: None
        """
        if len(self._physics_servers) < 2:
            err_control('Cannot call <start_all> for less than 2 instances.',
                   FONT.control)
            return
        for s_id in range(len(self._physics_servers)):
            # Dispatch to new threads
            t = multiprocessing.Process(target=self.start, args=(s_id,))
            self._process_pool.append(t)
            t.start()

    def start(self, server_id=0):
        """
        Kick start the simulation!
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id) to start.
        :return: None
        """
        # Get all handlers
        nruns, world, display, ctrl_handler, queue = self._physics_servers[server_id]

        # Kickstart the model, perform frame type check
        world.boot(display.info['frame'], job=display.info['engine']['job'])

        # Start control in another process
        ctrl_handler.run()

        # Run for given number of runs (used for # trajectories collection)
        for r in range(nruns):
            try:
                ctrl_handler.pause()

                # Clear the control queue
                while not queue.empty():
                    queue.get_nowait()

                # Preparing variables
                time_up, done, success = False, False, False
                self._init_time_stamp = time_util.get_abs_time()
                
                world.reset()
                status = display.run([t[1] for t in world.target])

                if status == -1:
                    err_control('Error loading simulation', FONT.control)
                    self.stop(server_id, -1)
                    break
                elif status == 1:
                    self.stop(server_id, 0)
                    logging.info_control('Replay finished. Exiting...', FONT.control)
                    continue
                elif status == 2:
                    logging.info_control('Start recording.', FONT.control)
                elif status == 3:
                    self.stop(server_id, 0)
                    logging.info_control('User quit during replay.', FONT.control)
                    break
                else:
                    logging.info_control('Display configs loaded. Starting simulation...',
                            FONT.control)

                # After loading and initialization finish, start rendering
                # (This can significantly boost performance)
                display.show()

                # Update initial states:
                self._control_update(world)

                # Next display states
                self._view_update(display)
                self._update_time_stamp = time_util.get_abs_time()

                # Finally start control loop (Core)
                ctrl_handler.resume()

                while not time_up and not done:
                    elt = time_util.get_elapsed_time(self._init_time_stamp)

                    time_since_last_update = time_util.get_elapsed_time(self._update_time_stamp)
                    self._update_time_stamp = time_util.get_abs_time()

                    # Perform control interruption first
                    if not queue.empty():
                        signal = queue.get_nowait()

                        self._control_interrupt(
                            world, display, signal,
                            time_since_last_update)

                    # Update model
                    time_up = world.update(elt)

                    # Check agent performance & task completion
                    done, success = world.check_states()

                    # TODO: GUI frame allow user to interact with the world
                    # dynamically, and vividly

                if success:
                    self.stop(server_id, 0)
                    logging.info_control('Task success! Exiting run {}...'.format(r),
                            FONT.disp)
                else:
                    self.stop(server_id, 1)
                    logging.info_control('Task failed! Exiting run {}...'.format(r),
                            FONT.disp)

            except KeyboardInterrupt:

                self.stop(server_id, -1)
                logging.info_control('User cancelled run {} by ctrl+c.'.format(r),
                        FONT.warning)
                quit_run = io_util.prompt(
                    'Do you wish to skip other runs and quit the program?'
                )
                if quit_run:
                    # Exit the program
                    self.exit(ctrl_handler, world, server_id)
                else:
                    continue

            except Exception:
                self.exit(ctrl_handler, world, server_id)
                
        self.exit(ctrl_handler, world, server_id)

    def stop(self, server_id, stop_status):
        """
        Hang the simulation.
        :param server_id: physics server id (a.k.a.
        simulation id, configuration id) to stop.
        :param stop_status: an integer indicating the status 
        of task completion, 0 for success, 1 for fail, and 
        -1 for error stop.
        :return: None
        """
        _, world, display, ctrl_handler, _ = self._physics_servers[server_id]
        display.close(stop_status)

    def exit(self, ctrl_handler, world, server_id=0):
        """
        Kill the simulation and exit, whatever the current status is. The
        difference from <stop> is that this method erases
        the given simulation from the program.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id)) to exit.
        :return: None
        """
        ctrl_handler.stop()
        self.stop(server_id, -1)

        # Destroy world
        world.clean_up()

        # TODO
        if self._process_pool:
            self._process_pool[server_id].terminate()
            self._process_pool[server_id] = None

        logging.info_control('Safe exit.', FONT.control)
        sys.exit(0)

    def _control_interrupt(self, world, display, signal, elapsed_time):
        """
        The control interruption, jumps to control defined
        in xml file, process the control signals and
        jumps back to the loop.
        :param world: the world model, provides tools
        :param display: the view model, set camera views
        :param signal: the signal received from control handler
        :return: None
        """
        # Only keep consistency for GUI usage
        elapsed_time = 1 if display.info['frame'] != 'gui' else elapsed_time * 50
        commands, instructions, view, update = \
            signal['cmd'], signal['instruction'], signal['camera'], signal['update']

        # Update view perspective modified by user pressing control key
        if update:
            self._view_update(display)
            return

        # Check for view perspective control
        if view:
            for move in view:
                mtype, delta = move

                # Update camera states from control input
                if mtype == 'pos':
                    rot_vec = display.get_camera_pose('rad')[1]
                    align_mat = math_util.euler2mat(math_util.vec((rot_vec[0], 0, rot_vec[1])))
                    translation = align_mat.dot(delta)
                    self._states['camera']['focus'] += translation * elapsed_time
                elif mtype == 'orn':
                    self._states['camera']['pitch'] += delta[0] * elapsed_time
                    self._states['camera']['yaw'] += delta[1] * elapsed_time
                else:
                    logging.info_control('Unrecognized view command type. Skipped',
                            FONT.ignore)
            # Apply state changes
            display.set_render_view(self._states['camera'])

        # Perform model / state control related functions
        if commands or instructions:
            tool = world.get_tool(signal['tid'], signal['key'])

            # First check if there's low level commands
            # A sequential list of commands to execute in order
            # These low level commands are set to absolute
            # values in all cases
            for cmd in commands:
                method, value = cmd
                if method == 'pos':
                    tool.tool_pos = value
                elif method == 'orn':
                    tool.tool_orn = value
                elif method == 'joint_states':
                    tool.joint_states = value
                elif method == 'pose':
                    tool.pinpoint(*value)
                else:
                    logging.info_control('Unrecognized control command type. Skipped',
                            FONT.ignore)

            # Next perform high level instructions
            for ins in instructions:
                # Use None for no value
                method, value = ins

                # Note this reset does not reset the elapsed
                # run time. The user is forced to finish the
                # task in limited amount of time.
                if method == 'rst' and value:
                    logging.info_control('Resetting...', FONT.model)
                    world.reset()

                    # Update the states
                    self._control_update(world)
                    logging.info_control('World is reset.', FONT.model)

                elif method == 'reach':
                    ### Note ###
                    # To achieve highly accurate, realistic control 
                    # experience, VR simulation requires absolute 
                    # position and relative orientation control, while 
                    # keyboard/phone uses relative position and 
                    # position control.

                    r_pos, r_orn = value

                    # Use relative pose control for devices
                    if display.info['frame'] != 'vr':

                        # Cartesian, quaternion
                        i_pos, i_orn = self._states['tool'][tool.tid]

                        # Orientation is always relative to the
                        # world frame, that is, absolute
                        r_mat = math_util.quat2mat(tool.orn)

                        if r_pos is not None:
                            # Scaling down the speed for robot arm
                            if tool.tid[0] == 'm':
                                r_pos /= 7.5

                            # Increment to get absolute pos
                            # Take account of rotation
                            i_pos += r_mat.dot(r_pos * elapsed_time)
                            tool.reach(i_pos, None)

                        if r_orn is not None:
                            
                            if tool.tid[0] == 'm':
                                end_orn_pos = math_util.vec(tool.joint_positions)\
                                    [tool.active_joints[-2:]]

                                i_orn = math_util.vec((end_orn_pos[1], end_orn_pos[0], 0))\
                                    + r_orn * elapsed_time
                                eef_joints = tool.active_joints[-2:]
                                joint_spec = tool.joint_specs

                                # Perform the clipping here
                                i_orn = math_util.clip_vec(
                                    math_util.vec((i_orn[1], i_orn[0])),
                                    math_util.vec(joint_spec['lower'])[eef_joints],
                                    math_util.vec(joint_spec['upper'])[eef_joints])
                                i_orn = math_util.vec((i_orn[0], i_orn[1], 0))

                                # Update the arm's orientation
                                self._states['tool'][tool.tid][1] = \
                                    math_util.vec((i_orn[1], i_orn[0], 0))
                            else:
                                i_orn += r_orn * elapsed_time

                                # Update the tool's orientation
                                self._states['tool'][tool.tid][1] = \
                                    math_util.vec(i_orn)
                            tool.reach(None, i_orn)

                            # Update the tool's position as orientation changes
                            self._states['tool'][tool.tid][0] = world.get_env_state(
                                ('tool', 'tool_pose'))[0][tool.tid][0]

                        pos_diff = tool.tool_pos - i_pos \
                            if tool.tid[0] == 'g' else tool.eef_pose[0] - i_pos

                        if math_util.rms(pos_diff) > tool.tolerance:
                            logging.info_control('Tool position out of reach. Set back.',
                                    FONT.warning)
                            state_pose = world.get_env_state(
                                ('tool', 'tool_pose'))[0][tool.tid]
                            self._states['tool'][tool.tid][0] = state_pose[0]
                    else:
                        threshold = 0.5

                        if tool.tid[0] == 'm':

                            # Joint 5, joint 6
                            end_orn_pos = math_util.vec(tool.joint_positions) \
                                [tool.active_joints[-2:]]

                            if math_util.rms(tool.tool_pos - r_pos) < threshold:
                                a_orn = math_util.vec((end_orn_pos[0], end_orn_pos[1], 0)) \
                                        + r_orn * elapsed_time
                                tool.reach(r_pos, None)#a_orn)
                        else:
                            i_orn = r_orn * elapsed_time + self._states['tool'][tool.tid][1]
                            tool.track(r_pos, i_orn, tool.traction)

                elif method == 'grasp':
                    tool.grasp(value)
                elif method == 'pick_and_place':
                    tool.pick_and_place(*value)

    def _view_update(self, display):
        """
        Display interruption, listens to display events and
        camera events, adjust view, as well as interact with
        the world in run time if necessary
        :param display: the view side of system
        :return: None
        """
        camera_pos, camera_orn = display.get_camera_pose(otype='deg')
        self._states['camera']['focus'] = camera_pos
        # self._states['camera']['flen'] = 1e-3
        self._states['camera']['pitch'] = camera_orn[0]
        self._states['camera']['yaw'] = camera_orn[1]

    def _control_update(self, world):
        """
        Update the internal control states of tool pose
        :param world: The world model that provides tools
        :return: None
        """
        init_states = world.get_env_state(('tool', 'tool_pose'))[0]
        # First control states
        for tid, init_pose in init_states.items():
            # if tid[0] == 'g':
            self._states['tool'][tid] = [
                init_pose[0],
                # Use radians
                math_util.quat2euler(init_pose[1])]
            # else:
            #     # For arms, use joint positions for tool orn;
            #     # The real orientation is end effector pose
            #     self._states['tool'][tid] = init_pose
