
from .world import World
from .view import View
from .adapter import Adapter
from .engine import physicsEngine
from .utils import io_util
import thread

# TODO: framework should only contain base class controller
# from .controller import Controller


class SimulationController(object):
    """
    The controller in MVC architecture.
    """
    def __init__(self, model_desc, view_desc,
                 config_batch):
        """
        Initialize the controller
        :param model_desc: the view part of the design,
        which refers to the control/display interface.
        :param view_desc: the model part of the design,
        which refers to the setup of simulation world.
        :param config_batch: the file path of configuration,
        in xml format. 
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
        loaded_async = True

        for i in range(len(configs)):
            conf = configs[i]
            assert i == conf.id, 'Error loading configuration: invalid id.'
            if not conf.async or not loaded_async \
                    or conf.view.frame != 'direct':
                print('Currently only support multiple instances '
                      'of non-GUI frame and asynchronous simulation. '
                      'GUI or synchronous frame can only run as '
                      'single simulation instance.')
                return
            # Update the comparator for the next checking
            loaded_async = conf.async
            # Perform loading
            self._physics_servers[conf.id] = self.load_config(conf)

            # step_size = 0.001,
            # job = 'run', async = False,
            # max_run_time = 10e6
            # Shared_Memory needs key as well!

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
        :return: loaded tuple (world, display, physics engine)
        """
        if conf.engine == 'bullet':
            pe = physicsEngine.BulletPhysicsEngine(
                conf.max_run_time, conf.frame,
                conf.job, conf.async, conf.step_size)
        elif conf.engine == 'mujoco':
            pe = physicsEngine.MujocoEngine()
        elif conf.engine == 'gazebo':
            pe = physicsEngine.GazeboEngine()
        else:
            raise NotImplementedError(
                'Currently only support three types of engines: '
                'Bullet Physics, Mujoco, and Gazebo.')

        world = World(conf.model_desc, pe)
        disp = View(conf.view_desc, Adapter(world), pe)

        return world, disp, pe

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
            t = thread.start_new_thread(self.start, (s_id,))
            self._thread_pool.append(t)

    def start(self, server_id=0):
        """
        Kick start the simulation!
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id) to start.
        :return: None
        """
        # First build the simulation
        world, display, _ = self._physics_servers[server_id]
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
        Kill the simulation, whatsoever the current status is.
        :param server_id: physics server id (a.k.a. simulation id,
        configuration id)) to kill.
        :return: None
        """
        pass

