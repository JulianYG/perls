from ..utils.io_util import FONT, loginfo

# TODO: check if there's a way to automatically parse


class ModelDebugger(object):

    def __init__(self, world):

        self._world = world

    @property
    def info(self):
        info_dic = self._world.info
        return info_dic

    @property
    def target(self):
        return self._world.target

    def build(self):
        self._world.build()
        loginfo('World Debugger info: \n{}'.format(self.info), FONT.model)

    def reset(self):
        self._world.reset()

    def update(self, elt):
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

    def get_states(self, *args):
        return self._world.get_states(*args)

    def boot(self, frame):
        return self._world.boot(frame)

    def notify_engine(self, stat):
        self._world.notify_engine(stat)

    def clean_up(self):
        self._world.clean_up()


class ViewDebugger(object):

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

    def update(self, camera, *_):
        self._display.update(camera)

    def load_config(self, desc):
        self._display.load_config(desc)

    def run(self, target=None):
        # while True:
        #     self._display._control_interrupt()
        #     self._display._engine.step(elapsed_time=0)
        #
        #     # print()
        return self._display.run(target)

    def close(self):
        self._display.close()