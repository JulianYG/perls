from .utils import io_util

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class View:
    """
    The display, view/rendering module of the system
    """
    
    def __init__(self, desc, adapter, graphics_engine):
        """
        Initialize the simulation display 
        :param desc: description file in xml format, 
        instructions about control/display settings
        :param adapter: adapter that allows view talking 
        to the world
        :param graphics_engine: Physics simulation physics_engine
        """
        self.name_str = 'ExampleBrowser'
        self._description = desc
        self._adapter = adapter
        self._engine = graphics_engine

        self._simulation_server_id = graphics_engine.ps_id
        self._frame = 'off'

    @property
    def info(self):
        """
        Get the basic info description of the 
        simulation display.
        :return: A dictionary of information of current view.
        {name, display frame, running simulation instances,
        physics_engine info}
        """
        return dict(
            name=self.name_str,
            frame=self._frame,
            server=self._simulation_server_id,
            engine=self._engine.info)

    def build(self):
        """
        Construct, configure and start the simulation display.
        Note this can only be called after world is instantiated,
        since it requires adapter to talk to the world.
        :return: None
        """
        self.load_config(self._description)

    def disable_hotkeys(self):
        """
        Special helper method to disable keyboard hot keys
        for convenience of operating GUI
        :return: None
        """
        self._engine.disable_hotkeys()

    def get_camera_pose(self, otype='quat'):
        """
        Get the current camera pose in rendering engine
        :return: (pos, orn) tuple based on given
        orientation type
        """
        return self._engine.get_camera_pose(otype=otype)

    def set_camera_pose(self, pos, orn):
        """
        Set the current camera pose in rendering engine
        :param pos: vec3 float cartesian position
        :param orn: vec4 float quaternion orientation
        :return: None
        """
        # TODO
        self._engine.set_camera_pose(pos, orn)

    def get_camera_image(self, itype='rgb'):
        """
        Get the camera snapshot of current scene
        :param itype: string of image type, can be
        'rgb', 'depth', 'segment'
        :return:
        rgb: an rgb array suitable for video
        depth: a depth value array of current env
        seg: segmentation value array
        """
        return self._engine.get_camera_image(itype=itype)

    def set_render_view(self, camera_info):
        """
        Update the view, mainly resetting the camera
        :param camera_info: dictionary of camera parameters.
        For GUI, camera_info is a dictionary of
        {flen: float, yaw: float degree,
        pitch: float degree, focus: float}
        For VR, camera_info is a tuple of
        (pos: vec3 cartesian, orn: vec4 quat)
        :return: None
        """
        if camera_info:
            self._engine.camera = camera_info

    def load_config(self, description):
        """
        Load given configuration file
        :param description: path string of
        display description xml file.
        :return: None
        """
        # Setup camera is for replay function
        camera_info, option_dic = io_util.parse_disp(description)

        # Configure display
        self._engine.configure_display(option_dic, camera_info)

    def run(self, targets=None):
        """
        Start the graphics render.
        Load simulation, and feed target objects in
        case of recording.
        :param targets: list of tracking body uids.
        :return: integer boot status. Only when loading
        returns 0, can program enter control loop.
        1 represents success of replay, and -1 represent
        error state.
        """
        status = self._engine.boot(targets)
        if status > -1:
            self._frame = self._engine.frame
        return status

    def close(self, exit_code):
        """
        Exit routine for display.
        :param exit_code: boolean indicating exit status for the task
        :return: None
        """
        self._engine.stop(exit_code)
