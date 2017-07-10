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
            name=self.name_str,
            tools=[t.name for t in self._tools.values()],
            tracking_bodies=[self._bodies[b].name for b in self._target_bodies],
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
        Get the tool list containing tools
        :return: list of Tool instances, can be 
        either grippers or arms, or even hands
        """
        return self._tools

    @property
    def target(self):
        """
        Get the list of tracked bodies in the world, 
        that is, being recorded.
        :return: A list of target bodies, this is 
        a subset of all bodies in the world
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
        for body in self._bodies.values():
            body.reset()

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
            self._target_bodies.append(body.uid)
        self._bodies[body.uid] = body

    def load_xml(self, file_name):
        """
        Load a given xml environment description file
        :param file_name: file path of this xml 
        :return: None
        """
        parse_tree = io_util.parse_env(file_name)
        self.name_str = parse_tree.env['title']

        # Load task completion checker
        self._checker = taskHandler.Checker(self._engine.ps_id, 
                                            self.name_str)

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

            # Tools are labeled by tool_id's
            self._tools[gripper_body.tid] = gripper_body
            self._bodies[gripper_body.uid] = gripper_body
            # Target bodies are listed as a bunch of uid's
            self._target_bodies.append(gripper_body.uid)

        for i in range(len(parse_tree.arm)):
            arm_spec = parse_tree.arm[i]

            assert i == arm_spec['id']
            gripper_spec = arm_spec['gripper']
            gripper_body = self.GRIPPER_TYPE[gripper_spec['type']](
                math_util.rand_bigint(),
                self._engine,
                path=gripper_spec['path'])
            gripper_body.name = gripper_spec['name']

            # Note here not appending gripper into tools since
            # we can only operate it through the arm
            self._target_bodies.append(gripper_body.uid)
            self._bodies[gripper_body.uid] = gripper_body

            arm_body = self.ARM_TYPE[arm_spec['type']](
                arm_spec['id'],
                self._engine,
                path=arm_spec['path'],
                pos=arm_spec['pos'], orn=arm_spec['orn'],
                null_space=arm_spec['null_space'],
                gripper=gripper_body)
            arm_body.name = arm_spec['name']

            self._tools[arm_body.tid] = arm_body
            self._bodies[arm_body.uid] = arm_body
            self._target_bodies.append(arm_body.uid)

        for asset in parse_tree.scene:
            asset_body = Body(self._engine,
                              asset['path'],
                              pos=asset['pos'],
                              orn=asset['orn'],
                              fixed=asset['fixed'])

            self._bodies[asset_body.uid] = asset_body
            if asset['record']:
                self._target_bodies.append(asset_body.uid)

        # TODO: Think if there's other stuff to conf
        # Add gravity after everything is loaded
        self._gravity = parse_tree.env['gravity']
        self._engine.configure_environment(self._gravity)

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
            tool = self._tools[self._tools.keys()[0]]
        else:
            tool = self._tools[t_id]
        # Mark the current using tool
        tool.mark = ('controlling', 2.5, (1.,0,0), None, .2)
        return tool

    def get_states(self, *args):
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

    def boot(self, frame):
        """
        Start the physics render.
        :param frame: string of frame type,
        for use of type check
        :return: render start state
        """
        return self._engine.start_engine(frame)

    def notify_engine(self, stat):
        """
        Set the status of render,
        :param stat: string, 'running', 'pending',
        'stopped', 'killed', 'finished', 'error'.
        :return: None
        """
        self._engine.status = stat

    def update(self, elp):
        """
        Update the states of the world.
        :return: None
        """
        return self._engine.step(elp)

    def clean_up(self):
        self._target_bodies = list()
        self._tools, self._bodies = dict(), dict()
        # Flush error messages
        for err_msg in self._engine.error:
            logerr(err_msg, FONT.model)
