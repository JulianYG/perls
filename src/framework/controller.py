
from .world import World
from .view import View
from .adapter import Adapter
from .engine import physicsEngine
from .utils import io_util
from .tool import debugger, tester
import threading

# TODO: framework should only contain base class controller
# from .controller import Controller


class SimulationController(object):
    """
    The controller in MVC architecture.
    """

    ENGINE_DIC = dict(bullet=physicsEngine.BulletPhysicsEngine,
                      mujoco=physicsEngine.MujocoEngine,
                      gazebo=physicsEngine.GazeboEngine)

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
             ... etc, all world/disp/engine related info},
         instance_id: ......}
        """
        info_dic = {}
        for s_id, (world, disp, pe) in self._physics_servers.items():
            info_dic[s_id] = dict(
                world_info=world.info,
                display_info=disp.info,
                engine_info=pe.info
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
            assert i == conf.id, 'Error loading configuration: invalid id.'

            # Perform loading
            world, disp, pe = self.load_config(conf)
            if num_configs > 1 and pe.info['real_time']:
                print('Currently only support multiple instances '
                      'of non-GUI frame and asynchronous simulation. '
                      'GUI or synchronous frame can only run as '
                      'single simulation instance. \nSimulation '
                      'configuration %d build skipped with error.' % i)
                pe.status = 'error'
            else:
                # TODO: Change all print statements to log.info/error
                print('Simulation configuration {} build success.'
                      'Build type: {}'.format(i, conf.build))

            self._physics_servers[conf.id] = (world, disp, pe)

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
        :return: loaded tuple (world, display, physics engine)
        """
        # Initialize physics engine
        pe = SimulationController.ENGINE_DIC[conf.engine](
            conf.id,
            conf.max_run_time, conf.job,
            conf.async, conf.step_size)
        world = World(conf.model_desc, pe)
        display = View(conf.view_desc, Adapter(world), pe)
        if conf.build == 'debug':
            world = debugger.ModelDebugger(world)
            display = debugger.ViewDebugger(display)
        elif conf.build == 'test':
            world = tester.ModelTester(world)
            display = tester.ViewTester(display)
        return world, display, pe

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
            print('Cannot call <start_all> for less than 2 instances.')
            return
        for s_id in range(len(self._physics_servers)):
            # Dispatch to new threads
            # t = thread.start_new_thread(self.start, (s_id,))
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
        # First build the simulation
        world, display, pe = self._physics_servers[server_id]
        display.build()
        world.build()
        # Finally start
        display.start()

    def stop(self, server_id=0):
        """
        Hang the simulation.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id) to stop. It acts as pause, 
        can use start to resume. 
        :return: None
        """
        pass

    def kill(self, server_id=0):
        """
        Kill the simulation, whatsoever the current status is. The 
        difference from <stop> is that this method erases 
        the given simulation from the program.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id)) to kill.
        :return: None
        """
        self._thread_pool[server_id] = None

