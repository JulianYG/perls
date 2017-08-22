

class Tester(object):

    def __init__(self, world, disp):

        pass


class ModelTester(Tester):

    @property
    def info(self):
        info_dic = self._world.info
        return info_dic

    @property
    def target(self):
        return self._world.target

    @property
    def gravity(self):
        """
        Get gravitational property of the world
        :return: vec3 float gravity
        """
        return self._world.gravity

    @property
    def traction(self):
        """
        Get the traction force on tracking bodies 
        in the world
        :return: float force in Newton
        """
        return self._world.traction

    @property
    def body(self):
        """
        Get the full list of bodies in the world
        :return: list of Body instances
        """
        return self._world.body

    @property
    def tool(self):
        """
        Get the tools inside the environment
        :return: dictionary of Tool instances, can be
        either grippers or arms, or even hands, where
        keys are tool ids, and values are tool instances
        """
        return self._world.tool

    def build(self):
        self._world.build()
        loginfo('World Debugger info: \n{}'.format(self.info), FONT.model)

    def reset(self):
        self._world.reset()

    def update(self, elt=0):
        self._world.update(elt)

    def load_body(self, file_path, pos, orn,
                  fixed=False, record=False):
        self._world.load_body(file_path, pos,
                              orn, record)

    def load_xml(self, file_name):
        self._world.load_xml(file_name)

    def get_tool(self, _id, key=None):
        tool = self._world.get_tool(_id, key)
        return tool

    def check_states(self):
        return self._world.check_states()

    def get_states(self, *args):
        return self._world.get_states(*args)

    def boot(self, frame):
        return self._world.boot(frame)

    def notify_engine(self, stat):
        self._world.notify_engine(stat)

    def clean_up(self):
        self._world.clean_up()


class ViewTester(object):

    def __init__(self, display):
        self._display = display

    @property
    def info(self):
        info_dic = self._display.info
        return info_dic

    def build(self):
        self._display.build()
        loginfo('Display Debugger info: \n{}'.format(self.info), FONT.disp)

    def disable_hotkeys(self):
        self._display.disable_hotkeys()

    def set_render_view(self, param):
        self._display.set_render_view(param)

    def get_camera_pose(self, otype='quat'):

        return self._display.get_camera_pose(otype=otype)

    def set_camera_pose(self, pos, orn):

        self._display.set_camera_pose(pos, orn)

    def get_camera_image(self, itype='rgb'):

        return self._display.get_camera_image(itype=itype)

    def load_config(self, desc):
        self._display.load_config(desc)

    def run(self, target=None):
        # while True:
        #     self._display._control_interrupt()
        #     self._display._engine.step(elapsed_time=0)
        #
        #     # print()
        return self._display.run(target)

    def close(self, exit_code):
        self._display.close(exit_code)
