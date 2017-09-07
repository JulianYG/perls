#!/usr/bin/env python

from .entity.body import Body
from .utils import io_util, math_util
from .utils.io_util import logerr, FONT
from .entity import PR2Gripper, rethinkGripper, WSG50Gripper
from .entity import sawyer, kuka
from .handler import taskHandler

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class World(object):
    """
    The basic scene setup in simulation 
    """
    GRIPPER_TYPE = dict(pr2=PR2Gripper.PR2Gripper,
                        rethink=rethinkGripper.RethinkGripper,
                        wsg=WSG50Gripper.WSG50Gripper)

    ARM_TYPE = dict(sawyer=sawyer.Sawyer, kuka=kuka.Kuka)

    # Storing inquiry legend
    _INQUIRY_DIC = dict(
        tool=['pose', 'v', 'omega', 'joint_states',
              'tool_pose',
              # TODO
              # 'force', 'wrench', 'shape',
              'name', 'contact'],
        body=['pose', 'v', 'omega',
              # 'force', 'wrench', 'shape',
              'name', 'tid',
              'contact'],
        env=['gravity', 'traction', 'target']
    )

    def __init__(self, desc_file, physics_engine):
        """
        Initialize default environment in simulation
        :param desc_file: assets description file
        :param physics_engine: :param physics_engine: Physics simulation physics_engine
        """
        # Default name before loading
        self.name_str = 'hello_world'
        self._description = desc_file
        self._engine = physics_engine
        self._target_bodies = list()
        self._tools, self._bodies = dict(), dict()

        # world specific attributes
        self._gravity = math_util.zero_vec(3)
        self._traction = 200
        self._checker = None

    @property
    def info(self):
        """
        Get the basic info description of the world.
        :return: A dictionary of information of this 
        world.
        {name, tools, tracking bodies, all bodies (assets).
        TODO: user activities, body time stamps}
        """
        return dict(
            task='_'.join(list(self.name_str)),
            tools=[t.name for t in self._tools.values()],
            tracking_bodies=self._target_bodies,
            assets=self._bodies.keys(),
            engine=self._engine.info
        )

    @property
    def gravity(self):
        """
        Get gravitational property of the world
        :return: vec3 float gravity
        """
        return self._gravity

    @property
    def traction(self):
        """
        Get the traction force on tracking bodies 
        in the world
        :return: float force in Newton
        """
        return self._traction

    @property
    def body(self):
        """
        Get the full list of bodies in the world
        :return: list of Body instances
        """
        return self._bodies

    @property
    def tool(self):
        """
        Get the tools inside the environment
        :return: dictionary of Tool instances, can be
        either grippers or arms, or even hands, where
        keys are tool ids, and values are tool instances
        """
        return self._tools

    @property
    def target(self):
        """
        Get the list of tracked bodies in the world, 
        that is, being recorded.
        :return: A list of target bodies, this is 
        a subset of all bodies in the world (name, uid)
        """
        return self._target_bodies

    def build(self):
        """
        Build the world
        :return: None
        """
        self.load_xml(self._description)

    def reset(self):
        """
        Reset the world to its initial conditions
        :return: None
        """
        for tool in self._tools.values():
            tool.reset()
            del tool.mark
        for body in self._bodies.values():
            body.reset()
            del body.mark
        self._engine.hold()
        
        # Fine tune the initial environment setup
        self._checker.initialize(self)
        self._engine.hold()

    def load_body(self, file_path, pos, orn,
                  fixed=False, record=False):
        """
        Load a given body at run time.
        :param file_path: body asset description file path
        :param pos: position to load the body, vec3 float cartesian
        :param orn: orientation vec4 float quaternion
        :param fixed: if using fixed base for the body, boolean
        :param record: if record info of this body, boolean
        :return: None
        """
        # TODO: depend on tool type
        body = Body(self._engine, file_path, pos, orn, fixed)
        if record:
            self._target_bodies.append((body.name, body.uid))
        self._bodies[body.name] = body

    def load_xml(self, file_name):
        """
        Load a given xml environment description file
        :param file_name: file path of this xml 
        :return: None
        """
        parse_tree = io_util.parse_env(file_name)
        self.name_str = (parse_tree.env['title'], parse_tree.scene_title)

        # Load task completion checker
        self._checker = taskHandler.Checker(self.name_str[1])

        for gripper in parse_tree.gripper:
            gripper_body = self.GRIPPER_TYPE[gripper['type']](
                gripper['id'],
                self._engine,
                path=gripper['path'],
                pos=gripper['pos'],
                orn=gripper['orn'])
            if gripper['fixed']:
                gripper_body.fix = (gripper['pos'], gripper['orn'])
            else:
                self._traction = gripper['traction']
                gripper_body.traction = self._traction
                gripper_body.hang()

            if gripper['attach']:
                children = self._load_asset(gripper['attach'])
                gripper_body.attach_children = \
                    ()

            # Tools are labeled by tool_id's
            self._tools[gripper_body.tid] = gripper_body
            self._bodies[gripper_body.name] = gripper_body

            # Target bodies are listed as a bunch of (uid, name) tuples
            self._target_bodies.append((gripper_body.name, gripper_body.uid))

        for i in range(len(parse_tree.arm)):
            arm_spec = parse_tree.arm[i]
            assert i == arm_spec['id']
            gripper_spec = arm_spec['gripper']

            # Only parse if gripper is there
            if gripper_spec:
                gripper_body = self.GRIPPER_TYPE[gripper_spec['type']](

                    # Use some random strings
                    hash(gripper_spec['name']),
                    self._engine,
                    path=gripper_spec['path'])
                gripper_body.name = gripper_spec['name']

                # Note here not appending gripper into tools since
                # we can only operate it through the arm
                self._target_bodies.append((gripper_body.name, gripper_body.uid))
                self._bodies[gripper_body.name] = gripper_body
            else:
                gripper_body = None

            arm_body = self.ARM_TYPE[arm_spec['type']](
                arm_spec['id'],
                self._engine,
                path=arm_spec['path'],
                pos=arm_spec['pos'], orn=arm_spec['orn'],
                collision_checking=arm_spec['collision_checking'],
                gripper=gripper_body)

            arm_body.name = arm_spec['name']
            self._tools[arm_body.tid] = arm_body
            self._bodies[arm_body.name] = arm_body
            self._target_bodies.append((arm_body.name, arm_body.uid))

        for asset in parse_tree.scene:
            bodies = self._load_asset(asset)

            for i in range(len(bodies) - 1):
                bodies[i]

        # TODO: Think if there's other stuff to conf
        # Add gravity after everything is loaded
        self._gravity = parse_tree.env['gravity']
        self._engine.configure_environment(self._gravity)

    def _load_asset(self, asset):
        """
        A helper function to load body from xml element
        :param asset: the element tree object to be loaded
        :return: A list of object uids in parent->children order
        """
        def _load_attachment():
            pass

        def _load_helper(p_elem, body_lst):

            asset_body = Body(self._engine,
                              p_elem['path'],
                              pos=p_elem['pos'],
                              orn=p_elem['orn'],
                              fixed=p_elem['fixed'])
            asset_body.name = p_elem['name']
            self._bodies[asset_body.name] = asset_body
            if p_elem['record']:
                self._target_bodies.append((asset_body.name, asset_body.uid))

            body_lst.append(asset_body)

            # Recursively add attached body
            if p_elem['attach']:
                _load_helper(p_elem['attach'], body_lst)

        bodies = []
        _load_helper(asset, bodies)
        return bodies

    def get_tool(self, _id, key=None):
        """
        Get the tool by id. Note this id corresponds to the
        id given in env xml file, and for robot arms, 
        keyboard numbering starts from 0,1,2,3,4,5,...
        User can also use ',' and '.' to cyclically go to the 
        last/next id in the same type of tools. 
        Note: User must specify at least one tool in xml.
        :param _id: integer id number
        :param key: character to choose from types of tools,
        For gripper or robot arm, hit 'g' before selecting numbers
        to choose from grippers, 'm' for arms, and 'h' 
        for hands, etc. Default is the first available tool.
        Currently supports up to 10 tools in total. (arms and 
        individual grippers)
        :return: the Tool instance associated with the id
        """
        t_id = '%s%d' % (key, _id)
        if not key or t_id not in self._tools:
            # print('Selected tool %s does not exist. Using default instead.' % t_id)
            tool = self._tools[list(self._tools.keys())[0]]
        else:
            tool = self._tools[t_id]
        # Mark the current using tool
        tool.mark = ('text', 2.5, (1., 0, 0), None,
                     .2, {'text': 'controlling'})
        return tool

    def get_env_state(self, *args):
        """
        Get world states by attribute name
        :param args: list of string tuples indicating what
        kind of inquiry (key), and what attributes
        to get (value).
        :return: states dictionary of given
        inquiry. Keys are ids, values are states.
        For example, [('body', 'pose')]
        It will return a list of dictionaries in the same
        order as inquiry list.
        """
        state_list = list()
        for key, value in args:
            assert key in self._INQUIRY_DIC, \
                'Invalid inquiry key \'%s\'' % key
            assert value in self._INQUIRY_DIC[key], \
                'Invalid inquiry value \'%s\'' % value
            if key == 'env':
                # Hope this is not evil as setattr!
                info = getattr(self, value)
            else:
                prop = getattr(self, key)
                info = dict((x, getattr(prop[x], value)) for x in prop)
            state_list.append(info)

        return state_list

    def get_task_state(self):
        """
        Get task states. Typically to get the task goal, or 
        other task relevant info.
        :return: states dictionary of task checker.
        """
        return self._checker.state

    def boot(self, frame, job='run'):
        """
        Start the physics render.
        :param frame: string of frame type,
        for use of type check
        :return: render start state
        """
        self._checker.set_job(job)
        status = self._engine.start_engine(frame)
        self._engine.hold(200)
        return status

    def notify_engine(self, stat):
        """
        Set the status of render,
        :param stat: string, 'running', 'pending',
        'stopped', 'killed', 'finished', 'error'.
        :return: None
        """
        self._engine.status = stat

    def check_states(self):
        """
        Check the current world state, see if 
        the task is completed.
        :return: tuple boolean <done, success> indicating whether 
        task is done, and whether it is successful.
        """
        return self._checker.check(self)

    def evaluate(self):
        """
        Evaluate the agents performance and generates a score
        :return: User defined score for agent
        """
        return self._checker.score(self)

    def update(self, elp=0, step_size=None):
        """
        Update the states of the world.
        :return: None
        """
        return self._engine.step(elp, step_size)

    def clean_up(self):
        """
        Clean up the environment and shut down
        :return: None
        """
        self._target_bodies = list()
        self._tools, self._bodies = dict(), dict()

        # Flush error messages
        for err_msg in self._engine.error:
            logerr(err_msg, FONT.model)

        self._engine.stop()
