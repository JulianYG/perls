from .utils import util, io_util
from .handler.eventHandler import NullHandler


class View:
    
    HANDLER_DIC = {}
    
    def __init__(self, desc, adapter, engine):
        """
        Initialize the simulation display 
        :param desc: description file in xml format, 
        instructions about control/display settings
        :param adapter: adapter that allows view talking 
        to the world
        :param engine: Physics simulation engine
        """
        self.name_str = 'Example View'
        self._description = desc
        self._adapter = adapter
        self._engine = engine
        self._init_time_stamp = None
        self._simulation_server_id = []
        self._frame = 'off'
        self._event_handler = NullHandler

    @property
    def info(self):
        """
        Get the basic info description of the 
        simulation display.
        :return: A dictionary of information of current view.
        {name, display frame, elapsed time, running simulation instances, 
        engine info}
        """
        return dict(
            name=self.name_str,
            frame=self._frame,
            run_time=util.get_elapsed_time(
                self._init_time_stamp),
            server=self._simulation_server_id,
            engine=self._engine.info)

    def build(self):
        """
        Construct the display 
        :return: Physics server ID.
        """
        status = self._engine.load_simulation()
        self._init_time_stamp = util.get_time_stamp()
        return

    def start(self):
        """
        Configure and start the simulation display
        :return: None
        """
        self.name_str, frame_info, option_dic, control_type = \
            io_util.parse_disp(self._description)
        self._frame = frame_info[0]
        self._engine.configure_display(frame_info, option_dic)
        elapsed_time = util.get_elapsed_time(
                self._init_time_stamp)
        done = False
        while not done:
            done = self._engine.step()


    def control_interrupt(self):
        pass
        # event_handler =
        # cmd_handler =


