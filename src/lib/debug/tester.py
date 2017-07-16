
class Tester(object):

    def __init__(self, world, disp):

        pass


class ModelTester(Tester):

    def __init__(self, model):
        self._world = model

    def build(self):
        return self._world.build()

    def reset(self):
        return self._world.reset()

    def load_body(self, file_path, pos, orn,
                  fixed=False, record=False):
        self._world.load_body(file_path, pos,
                              orn, record)

    def load_xml(self, file_name):
        self._world.load_xml(file_name)

    def get_tool(self, _id, key=None):
        return self._world.get_tool(_id, key)


class ViewTester(Tester):

    def __init__(self, view):
        self._display = view




