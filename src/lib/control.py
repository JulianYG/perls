import threading

from .state import physicsEngine, robotEngine
from .adapter import Adapter
from .render import graphicsEngine, camera
from .handler.base import NullHandler
from .handler.eventHandler import InteractiveHandler
from .handler.controlHandler import (KeyboardEventHandler,
                                     ViveEventHandler,
                                     AppEventHandler)
from .debug import debugger, tester
from .utils import io_util, time_util, math_util
from .utils.io_util import (FONT,
                            loginfo,
                            logerr)
from .view import View
from .world import World

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


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
        gazebo=physicsEngine.OpenRaveEngine,

        # Reality
        intera=robotEngine.InteraEngine,
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
        self._config = config_batch
        self._physics_servers = dict()
        self._thread_pool = list()

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
            world, disp, ctrl_hdlr = self.load_config(conf)
            if num_configs > 1 and (not conf.async):
                logerr('Currently only support multiple instances '
                       'of non-GUI frame and asynchronous simulation. '
                       'GUI or synchronous frame can only run as '
                       'single simulation instance. \nSimulation '
                       'configuration %d build skipped with error. ' % i,
                       FONT.control)
                world.notify_engine('error')
            else:
                loginfo('Simulation configuration {} build success. '
                        'Build type: {}'.format(i, conf.build),
                        FONT.control)
                world.notify_engine('pending')
            self._physics_servers[conf.id] = (world, disp, ctrl_hdlr)

    @staticmethod
    def load_config(conf):
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
            log_dir=conf.log
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
            pe.ps_id, conf.sensitivity, conf.rate
        )

        # TODO
        if conf.build == 'debug':
            world = debugger.ModelDebugger(world)
            display = debugger.ViewDebugger(display)
        elif conf.build == 'test':
            world = tester.ModelTester(world)
            display = tester.ViewTester(display)

        # Give record name for physics physics_engine
        if conf.job == 'record' or conf.job == 'replay':
            ge.record_name = \
                conf.record_name or \
                '{}_{}'.format(world.info['name'],
                               display.info['name'])

        # connect to bullet graphics/display render server,
        # Build display first to load world faster
        display.build()
        world.build()

        # Special case for keyboard control on View side
        if conf.control_type == 'keyboard':
            # Disable keyboard shortcuts for keyboard control
            display.disable_hotkeys()

        return world, display, ctrl_handler

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
            logerr('Cannot call <start_all> for less than 2 instances.',
                   FONT.control)
            return
        for s_id in range(len(self._physics_servers)):
            # Dispatch to new threads
            t = threading.Thread(target=self.start, args=(s_id,))
            self._thread_pool.append(t)
            t.start()

    def start(self, server_id=0):
        """
        Kick start the simulation!
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id) to start.
        :return: None
        """
        # Get all handlers
        world, display, ctrl_handler = self._physics_servers[server_id]

        # Preparing variables
        time_up, done, success = False, False, False
        self._init_time_stamp = time_util.get_abs_time()

        track_targets = world.get_states(('env', 'target'))[0]

        # Pass in targets uids
        status = display.run([t[0] for t in track_targets])

        if status == -1:
            logerr('Error loading simulation', FONT.control)
            self.stop(server_id)
            return
        elif status == 1:
            self.stop(server_id)
            loginfo('Replay finished. Exiting...', FONT.control)
            return
        elif status == 2:
            loginfo('Start recording.', FONT.control)
        else:
            loginfo('Display configs loaded. Starting simulation...',
                    FONT.control)

        # Kickstart the model, perform frame type check
        world.boot(display.info['frame'])

        # Update initial states:
        self._control_update(world)

        # Next display states
        self._view_update(display)

        self._update_time_stamp = time_util.get_abs_time()

        # Finally start control loop (Core)
        try:
            while not time_up or done:

                elt = time_util.get_elapsed_time(self._init_time_stamp)

                time_since_last_update = time_util.get_elapsed_time(self._update_time_stamp)
                self._update_time_stamp = time_util.get_abs_time()

                # Perform control interruption first
                self._control_interrupt(
                    world, display, 
                    ctrl_handler.signal, time_since_last_update)

                # Update model
                time_up = world.update(elt)

                # Lastly check task completion, communicate
                # with the model
                # TODO
                # done, success = self._checker_interrupt()

            if success:
                loginfo('Task success! Exiting simulation...',
                        FONT.disp)
            else:
                loginfo('Task failed! Exiting simulation...',
                        FONT.disp)
        except KeyboardInterrupt:
            loginfo('User exits the program by ctrl+c.',
                    FONT.warning)
            self.stop(server_id)

    def stop(self, server_id):
        """
        Hang the simulation.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id) to stop. It acts as pause,
        can use start to resume.
        :return: None
        """
        world, display, ctrl_handler = self._physics_servers[server_id]

        ctrl_handler.stop()
        world.clean_up()
        display.close()

        loginfo('Safe exit.', FONT.control)

    def kill(self, server_id=0):
        """
        Kill the simulation, whatsoever the current status is. The
        difference from <stop> is that this method erases
        the given simulation from the program.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id)) to kill.
        :return: None
        """
        # TODO
        self._thread_pool[server_id] = None

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
                    loginfo('Unrecognized view command type. Skipped',
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
                    loginfo('Unrecognized control command type. Skipped',
                            FONT.ignore)

            # Next perform high level instructions
            for ins in instructions:
                # Use None for no value
                method, value = ins

                # Note this reset does not reset the elapsed
                # run time. The user is forced to finish the
                # task in limited amount of time.
                if method == 'rst' and value:
                    loginfo('Resetting...', FONT.model)
                    world.reset()

                    # Update the states
                    self._control_update(world)
                    loginfo('World is reset.', FONT.model)

                elif method == 'reach':

                    # Cartesian, quaternion
                    r_pos, r_orn = value
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
                        tool.pinpoint(i_pos, tool.tool_orn)

                    if r_orn is not None:
                        ###
                        # Can try directly setting joint states here
                        i_orn += r_orn * elapsed_time
                        if tool.tid[0] == 'm':
                            eef_joints = tool.active_joints[-2:]
                            joint_spec = tool.joint_specs

                            # Perform the clipping here
                            i_orn = math_util.clip_vec(
                                math_util.vec((i_orn[1], i_orn[0])),
                                math_util.vec(joint_spec['lower'])[eef_joints],
                                math_util.vec(joint_spec['upper'])[eef_joints])
                            i_orn = math_util.vec((i_orn[0], i_orn[1], 0))

                            # Update the tool's orientation
                            self._states['tool'][tool.tid][1] = \
                                math_util.vec((i_orn[1], i_orn[0], 0))
                        else:

                            # Update the tool's orientation
                            self._states['tool'][tool.tid][1] = \
                                math_util.vec(i_orn)

                        tool.pinpoint(tool.tool_pos, i_orn)

                        # Update the tool's position as orientation changes
                        self._states['tool'][tool.tid][0] = world.get_states(
                            ('tool', 'tool_pose'))[0][tool.tid][0]

                    pos_diff = tool.tool_pos - i_pos

                    if math_util.rms(pos_diff) > tool.tolerance:
                        loginfo('Tool position out of reach. Set back.',
                                FONT.warning)
                        state_pose = world.get_states(
                            ('tool', 'tool_pose'))[0][tool.tid]
                        self._states['tool'][tool.tid][0] = state_pose[0]

                elif method == 'grasp':
                    tool.grasp(value)
                elif method == 'pick_and_place':
                    tool.pick_and_place(*value)

                    # TODO: GUI frame allow user to interact with the world
                    # # dynamically, and vividly
                    # if self._frame == 'gui':
                    #     info = self._event_handler.signal
                    #     if info:
                    #         self._adapter.update_world(info)

    def _view_update(self, display):
        """
        Display interruption, listens to display events and
        camera events, adjust view, as well as interact with
        the world in run time if necessary
        :param display: the view side of system
        :return: None
        """
        camera_pos, camerq_orn = display.get_camera_pose(otype='deg')
        self._states['camera']['focus'] = camera_pos
        # self._states['camera']['flen'] = 1e-3
        self._states['camera']['pitch'] = camerq_orn[0]
        self._states['camera']['yaw'] = camerq_orn[1]

    def _control_update(self, world):
        """
        Update the internal control states of tool pose
        :param world: The world model that provides tools
        :return: None
        """
        init_states = world.get_states(('tool', 'tool_pose'))[0]
        # First control states
        for tid, init_pose in init_states.items():
            self._states['tool'][tid] = [
                init_pose[0], 
                # Use radians
                math_util.quat2euler(init_pose[1])]

    def _checker_interrupt(self, signal):
        """
        Checker interrupt, performs checking on current
        world states and report status.
        :return: (done, success) tuple, where 'done' is
        boolean and 'success' is a scalar, either 0/1
        binary, or float in [0,1] representing quality.
        """
        status = self._adapter.check_world_states()
        return status