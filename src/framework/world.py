# __package__ = 'sim_.simulation'

from .object.body import Body
from .utils import io_util
from .object import PR2Gripper, rethinkGripper, WSG50Gripper
from .object import sawyer, kuka

class World(object):

    GRIPPER_TYPE = dict(pr2=PR2Gripper.PR2Gripper,
                       rethink=rethinkGripper.RethinkGripper,
                       wsg=WSG50Gripper.WSG50Gripper)

    ARM_TYPE = dict(sawyer=sawyer.Sawyer, kuka=kuka.Kuka)

    """
    The basic scene setup in simulation 
    """
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
        self.tools = list()
        self.bodies = dict()

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
        :return: 0 if success, -1 if failure
        """
        return self.load_xml(self._description)

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

        # TODO: Think if there's other stuff to conf
        self._engine.configure_environment(parse_tree.env['gravity'])

        for gripper in parse_tree.gripper:
            gripper_body = self.GRIPPER_TYPE[gripper['type']](
                self._engine,
                path=gripper['path'],
                pos=gripper['pos'], orn=gripper['orn']
            )
            self.bodies[gripper_body.uid] = gripper_body
            self.tracking_bodies.append(gripper_body.uid)

        for arm in parse_tree.arm:
            gripper_spec = arm['gripper']
            gripper_body = self.GRIPPER_TYPE[gripper_spec['type']](
                self._engine,
                path=gripper_spec['path']
            )
            self.bodies[gripper_body.uid] = gripper_body

            # TODO: confirm this is necessary
            self.tracking_bodies.append(gripper_body.uid)
            arm_body = self.ARM_TYPE[arm['type']](
                self._engine,
                path=arm['path'],
                pos=arm['pos'], orn=arm['orn'],
                null_space=False,
                gripper=gripper_body
            )
            self.bodies[arm_body.uid] = arm_body
            self.tracking_bodies.append(arm_body.uid)

        for asset in parse_tree.scene:
            asset_body = Body(self._engine,
                              asset['path'],
                              pos=asset['pos'], orn=asset['orn'],
                              fixed=asset['fixed'])
            self.bodies[asset_body.uid] = asset_body
            if asset['record']:
                self.tracking_bodies.append(asset_body.uid)

