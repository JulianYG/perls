# __package__ = 'sim_.simulation'

from .object.body import Body
from .utils import io_util, math_util
from .object import PR2Gripper, rethinkGripper, WSG50Gripper
from .object import sawyer, kuka


class World(object):
    """
    The basic scene setup in simulation 
    """

    GRIPPER_TYPE = dict(pr2=PR2Gripper.PR2Gripper,
                       rethink=rethinkGripper.RethinkGripper,
                       wsg=WSG50Gripper.WSG50Gripper)

    ARM_TYPE = dict(sawyer=sawyer.Sawyer, kuka=kuka.Kuka)

    def __init__(self, desc_file, engine):
        """
        Initialize default environment in simulation
        :param desc_file: assets description file
        :param engine: :param engine: Physics simulation engine
        """
        # Default name before loading
        self.name_str = 'hello_world'
        self._description = desc_file
        self._engine = engine

        self.tracking_bodies = list()
        self.tools, self.bodies = dict(), dict()

    @property
    def info(self):
        """
        Get the basic info description of the world.
        :return: A dictionary of information of this 
        world.
        {name, tools, tracking bodies, all bodies (assets).
        TODO: user activities, body time stamps}
        """
        return dict(name=self.name_str,
                    tools=[t.name for t in self.tools],
                    tracking_bodies=[b.name for b in self.tracking_bodies],
                    assets=self.bodies.keys(),
                    )

    def build(self):
        """
        Build the world
        :return: None
        """
        self.load_xml(self._description)

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
        #TODO: depend on tool type
        body = Body(self._engine, file_path, pos, orn, fixed)
        if record:
            self.tracking_bodies.append(body.uid)
        self.bodies[body.uid] = body

    def load_xml(self, file_name):
        """
        Load a given xml environment description file
        :param file_name: file path of this xml 
        :return: None
        """
        parse_tree = io_util.parse_env(file_name)
        self.name_str = parse_tree.env['title']
        for gripper in parse_tree.gripper:
            gripper_body = self.GRIPPER_TYPE[gripper['type']](
                gripper['tid'],
                self._engine,
                path=gripper['path'],
                pos=gripper['pos'],
                orn=gripper['orn'])

            if gripper['fixed']:
                gripper_body.fix = (gripper['pos'], gripper['orn'])

            self.tools['g%d' % gripper_body.tid] = gripper_body

            self.tracking_bodies.append(gripper_body.uid)

        for arm in parse_tree.arm:
            gripper_spec = arm['gripper']
            gripper_body = self.GRIPPER_TYPE[gripper_spec['type']](
                math_util.rand_bigint(),
                self._engine,
                path=gripper_spec['path'])
            # Note here not appending gripper into tools since
            # we can only operate it through the arm
            # TODO: confirm this is necessary
            self.tracking_bodies.append(gripper_body.uid)
            arm_body = self.ARM_TYPE[arm['type']](
                arm['tid'],
                self._engine,
                path=arm['path'],
                pos=arm['pos'], orn=arm['orn'],
                null_space=False,
                gripper=gripper_body)

            self.tools['m%d' % arm_body.tid] = arm_body

            self.tracking_bodies.append(arm_body.uid)

        for asset in parse_tree.scene:
            asset_body = Body(self._engine,
                              asset['path'],
                              pos=asset['pos'],
                              orn=asset['orn'],
                              fixed=asset['fixed'])

            self.bodies[asset_body.name] = asset_body
            if asset['record']:
                self.tracking_bodies.append(asset_body.uid)

        # TODO: Think if there's other stuff to conf
        # Add gravity after everything is loaded
        self._engine.configure_environment(parse_tree.env['gravity'])

    def get_tool(self, t_id, key=None):
        """
        Get the tool by id. Note this id corresponds to the 
        id given in env xml file, and for robot arms, 
        keyboard numbering starts from 0,1,2,3,4,5,...
        User can also use ',' and '.' to cyclically go to the 
        last/next id in the same type of tools. 
        Note: User must specify at least one tool in xml.
        :param t_id: integer id number
        :param key: character to choose from types of tools,
        For gripper or robot arm, hit 'g' before selecting numbers
        to choose from grippers, 'm' for arms, and 'h' 
        for hands, etc. Default is the first available tool. 
        Currently supports up to 10 tools in total. (arms and 
        individual grippers)
        :return: the Tool instance associated with the id
        """
        if key:
            tool = self.tools['%s%d' % (key, t_id)]
        else:
            tool = self.tools[self.tools.keys()[0]]
        # Mark the current using tool
        tool.mark = ('controlling', 2.5, (1.,0,0), None, 1.)
        return tool

