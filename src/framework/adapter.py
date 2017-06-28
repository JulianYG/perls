
class Adapter(object):
    """
    In this project only view2model adapter
    is needed, since model directly 
    writes into view
    """
    def __init__(self, model):

        self._model = model

    def update(self):
        pass

    def add_body(self):

        pass

    def change_body(self):

        pass




