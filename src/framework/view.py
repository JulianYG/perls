from .utils import util, io_util
from .handler.base import NullHandler
from .handler.controlHandler import (KeyboardEventHandler,
                                     ViveEventHandler,
                                     AppEventHandler,
                                     CmdEventHandler)
from .handler.eventHandler import AssetHandler


class View:
    """
    The view part of the model
    """
    HANDLER_DIC = dict(keyboard=KeyboardEventHandler,
                       vive=ViveEventHandler,
                       phone=AppEventHandler,
                       off=NullHandler)
    
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
        self._control_type = 'off'
        self._control_handler = NullHandler()
        self._event_handler = AssetHandler()

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
            control=self._control_type,
            run_time=util.get_elapsed_time(
                self._init_time_stamp),
            server=self._simulation_server_id,
            engine=self._engine.info)

    def build(self):
        """
        Construct the display 
        :return: None
        """
        self.name_str, frame_info, camera_info, option_dic, \
            self._control_type, sensitivity, rate = \
            io_util.parse_disp(self._description)
        self._frame = frame_info[0]
        # Set up control event interruption handlers
        # TODO
        self._control_handler = self.HANDLER_DIC[self._control_type](
            sensitivity, rate
        )

        # Special case for keyboard control on View side
        if self._control_type == 'keyboard':
            # Disable keyboard shortcuts for keyboard control
            option_dic['keyboard_shortcut'] = False

        # Configure display, connect to bullet physics server
        self._engine.configure_display(frame_info, camera_info, option_dic)

    def start(self):
        """
        Configure and start the simulation display.
        Note this can only be called after world is built,
        since it requires adapter to talk to the world.
        :return: None
        """
        # Boot the adapter to talk with world (model)
        self._adapter.reset()

        # Some preparation jobs for control
        self._engine.load_simulation()
        done = False
        self._init_time_stamp = util.get_abs_time()

        # Start control loop
        while not done:
            elt = util.get_elapsed_time(self._init_time_stamp)

            # Run into interruption
            self._control_interrupt()
            done = self._engine.step(elapsed_time=elt)

    def _control_interrupt(self):

        signal = self._control_handler.signal
        self._adapter.react(signal)

        # GUI frame allow user to interact with the world
        # dynamically, and vividly
        if self._frame == 'gui':
            info = self._event_handler.signal
            if info:
                self._adapter.update_world(info)
