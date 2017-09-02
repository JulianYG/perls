# !/usr/env/bin python

import threading

from .state import physicsEngine, robotEngine
from .adapter import Adapter
from .render import graphicsEngine, camera
from ..comm.worker import StateReader, StateLogger
from .debug import debugger, tester
from .utils import io_util, time_util
from .utils.io_util import FONT, loginfo, logerr
from .view import View
from .world import World

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class PerlsServer(object):
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

        self._init_time_stamp = None
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
            world, disp = self.load_config(conf)
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
            self._physics_servers[conf.id] = (
                world, disp,
                # Database reader/writer ids
                StateReader(conf.id),
                StateLogger(conf.id))

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
        ge = PerlsServer._GRAPHICS_ENGINES[conf.graphics_engine](
            conf.disp_info,
            conf.job,
            conf.video,
            log_dir=conf.log
        )

        # Initialize physics render (state render)
        pe = PerlsServer._PHYSICS_ENGINES[conf.physics_engine](
            conf.id,
            ge.ps_id,
            conf.max_run_time,
            conf.async,
            conf.step_size,
        )

        world = World(conf.model_desc, pe)
        display = View(conf.view_desc, Adapter(world), ge)

        # TODO
        if conf.build == 'debug':
            world = debugger.ModelDebugger(world)
            display = debugger.ViewDebugger(display)
        elif conf.build == 'test':
            world = tester.ModelTester(world)
            display = tester.ViewTester(display)

        # connect to bullet graphics/display render server,
        # Build display first to load world faster
        display.build()
        world.build()

        return world, display

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
        world, display, reader, writer = self._physics_servers[server_id]

        # Preparing variables
        time_up = False
        self._init_time_stamp = time_util.get_abs_time()

        track_targets = world.get_states(('env', 'target'))[0]
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

        # Kick start the model, perform frame type check
        world.boot(display.info['frame'])

        # Finally start control loop
        try:
            while not time_up:
                elt = time_util.get_elapsed_time(self._init_time_stamp)

                # Read desired states
                reader.consume(world, display)

                # Update model
                time_up = world.update(elt)

                # Update actual states
                writer.produce(world)

                # Next check task completion, communicate
                # with the model
                # TODO This should happen in client, not here
                # done, success = self._checker_interrupt()

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
        world, display, reader, writer = self._physics_servers[server_id]

        reader.stop()
        writer.stop()
        world.clean_up()
        display.close()

        loginfo('Safe exit.', FONT.control)

    def kill(self, server_id=0):
        """
        Kill the simulation, whatsoever the current status is. The
        difference from <stop> is that this method erases
        the given simulation from the program.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id)) to exit.
        :return: None
        """
        # TODO
        self._thread_pool[server_id] = None
