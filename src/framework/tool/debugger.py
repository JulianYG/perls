

class ModelDebugger(object):

    def __init__(self, world):

        self._world = world

    @property
    def info(self):
        return self._world.info

    def build(self):
        self._world.build()

    def reset(self):
        self._world.reset()

    def load_body(self, file_path, pos, orn,
                  fixed=False, record=False):
        self._world.load_body(file_path, pos,
                              orn, record)

    def load_xml(self, file_name):
        self._world.load_xml(file_name)

    def get_tool(self, _id, key=None):

        tool = self._world.get_tool(_id, key)

        return tool


class ViewDebugger(object):

    def __init__(self, display):
        self._display = display

    @property
    def info(self):
        return self._display.info

    def build(self):
        self._display.build()

    def start(self):
        # Boot the adapter to talk with world (model)
        self._display._adapter._update_states()

        # Some preparation jobs for control
        self._display._engine.load_simulation()

        self.run()

    def run(self):

        while True:
            self._display._control_interrupt()
            self._display._engine.step(elapsed_time=0)

            # print()
