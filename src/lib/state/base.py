import abc


class StateEngine(object):

    _STATUS = \
        ['running', 'pending', 'stopped', 'off',
         'killed', 'finished', 'error']

    def __init__(self, e_id, max_run_time):
        """
        Constructor for base class state render
        :param e_id: integer, to indicate label
        of this render
        :param max_run_time: maximum running time.
        """
        self.engine_id = e_id
        self._max_run_time = max_run_time
        self._status = self._STATUS[3]
        self._error_message = list()

    @abc.abstractproperty
    def version(self):
        """
        Get the version of this state render
        :return: version string
        """
        return NotImplemented

    @abc.abstractproperty
    def info(self):
        """
        Get the basic info description of render.
        :return: A dictionary of information
        {version/name, status, real time, id,
        step size (if async), max running time}
        """
        return NotImplemented

    @property
    def error(self):
        """
        Get error messages
        :return: list of strings of msgs
        """
        return self._error_message

    @property
    def status(self):
        """
        Get current running status string
        :return: a string from
        ['running', 'pending', 'stopped', 'off',
         'killed', 'finished', 'error']
        """
        return self._status

    @status.setter
    def status(self, stat):
        """
        Set the state render status by given stat string
        :param stat: string of status
        :return: None
        """
        self._status = stat

    @abc.abstractmethod
    def load_asset(self, file_path, pos, orn, fixed):
        """
        Load the mesh asset of an entity (body) into the world.
        File typically given in urdf/sdf/xml format.
        :param file_path: absolute or relative path of asset file
        :param pos: position vec3 float cartesian
        :param orn: orientation vec4 float quaternion tuple
        :param fixed: boolean indicating whether the body should
        be fixed to given pose when loaded.
        :return: integer of body unique id, body link indices,
        body joint indices
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_scene_position(self, uid):
        """
        Get body position in the scene
        :param uid: integer body unique id
        :return: position vec3 float cartesian
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_scene_orientation(self, uid, otype='quat'):
        """
        Get body orientation in the scene
        :param uid: integer body unique id
        :param otype: orientation type, can be specified as
        'quat' for quaternion (vec4 float),
        'deg' for euler degrees (vec3 float), or
        'rad' for euler radians (vec3 float).
        :return: orientation vector in given otype
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_camera_position(self, uid, camera_pos, camera_orn):
        """
        Get body position in the camera frame
        :param uid: integer body unique id
        :param camera_pos: vec3 float cartesian camera
        position in the world frame.
        :param camera_orn: vec4 float quaternion camera
        orientation in the world frame.
        :return: position vec3 float cartesian
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_camera_orientation(
            self, uid, camera_pos, camera_orn, otype):
        """
        Get body orientation in the camera frame
        :param uid: integer body unique id
        :param camera_pos: vec3 float cartesian camera
        position in the world frame.
        :param camera_orn: vec4 float quaternion camera
        orientation in the world frame.
        :param otype: orientation type, can be specified as
        'quat' for quaternion (vec4 float),
        'deg' for euler degrees (vec3 float), or
        'rad' for euler radians (vec3 float).
        :return: orientation vector in given otype
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_relative_pose(self, uid, frame_pos, frame_orn):
        """
        Get the body's position and orientation in the
        specified coordinate system.
        :param uid: body's unique integer id
        :param frame_pos: origin of given frame (vec3 float
         cartesian), relative to the world frame
        :param frame_orn: orientation of given frame (vec4
        float quaternion), relative to the world frame
        :return: (pos, orn) of querying object in given frame
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_scene_pose(self, uid, pos, orn):
        """
        Set position and orientation (vec4 float quaternion),
        of specified body
        :param uid: integer body unique id
        :param pos: vec3 float cartesian position vector
        :param orn: vec4 float quaternion orientation vector
        :return: 0 if success, -1 if failed
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_name(self, uid):
        """
        Get the name string of the body
        :param uid: integer body unique id
        :return: string of name
        """
        return NotImplemented

    @abc.abstractmethod
    def get_link_name(self, uid, lid):
        """
        Get the name string of given link
        :param uid: integer body unique id
        :param lid: integer link id of body
        :return: name string of link on body
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_visual_shape(self, uid):
        """
        Get the visual shape of body
        :param uid: integer body unique id
        :return: A list of visual shape data:
        [linkIndex, visualGeometryType, dimensions (vec3),
        meshAssetFileName (str), localVisualFramePos (vec3),
        localVisualFrameOrn (vec4), rgbaColor (vec4)]
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_visual_shape(self, uid, qid, shape):
        """
        Set the body visual shape given parameters.
        :param uid: integer body unique id
        :param qid: link index of body to be changed, -1 for base
        :param shape: string for shape,
        choose among 'sphere', 'box', 'capsule', 'cylinder',
        'plane', and 'mesh'.
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_visual_color(self, uid, qid, color, spec=False):
        """
        Set the body visual color given parameters.
        :param uid: integer body unique id
        :param qid: link index of body to be changed, -1 for base
        :param color: vec4 float (r, g, b, alpha) in [0, 1] RGBA
        if spec=False, or vec3 float specular color components,
        RED, GREEN and BLUE, can be from 0 to large number (>100) if
        spec=True
        :param spec: boolean indicating whether specular color or not
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_texture(self, uid, qid, texture):
        """
        Set the body texture with given file
        :param uid: integer body unique id
        :param qid: link index of body to be changed, -1 for base
        :param texture: file path of texture, typcially
        jpg or png format.
        :return:
        """
        return NotImplemented

    @abc.abstractmethod
    def change_loaded_texture(self, texture_id, pixels, w, h):
        """
        Change the loaded texture
        :param texture_id: integer id of loaded texture
        :param pixels: full array of pixel values, for example,
        [255] * width * height * 3
        :param w: integer width of region
        :param h: integer height of region
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_linear_velocity(self, uid):
        """
        Get the linear velocity of body in world frame
        :param uid: body unique id
        :return: vec3 float cartesian linear velocity
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_linear_velocity(self, uid, vel):
        """
        Set the body linear velocity
        :param uid: integer body unique id
        :param vel: vec3 float of linear velocity
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_angular_velocity(self, uid):
        """
        Get the angular velocity of body in world frame
        :param uid: body unique id
        :return: vec3 float cartesian angular velocity
        in radian
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_angular_velocity(self, uid, vel):
        """
        Set the body angular velocity
        :param uid: integer body unique id
        :param vel: vec3 float radian of angular velocity
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_link_state(self, uid, lid):
        """
        Get the state of given link on given body
        :param uid: integer body unique id
        :param lid: integer link id on body
        :return: a tuple of
        link world position (vec3 float cartesian),
        link world orientation (vec4 float quaternion),
        local position offset of CoM frame,
        local orientation offset of CoM frame,
        world position of asset file link frame,
        world orientation of asset file link frame,
        cartesian link world linear velocity,
        radian link world angular velocity.
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_joint_info(self, uid, jid):
        """
        Get the information of body joint
        :param uid: integer unique body id
        :param jid: integer joint id on body
        :return: a tuple of
        joint index,
        joint name string in asset file,
        joint type string ('fixed', 'revolute',
        'prismatic', 'spherical', 'planar', 'gear',
        'point2point'),
        the first position index in the positional
        state variables for this body,
        the first velocity index in the velocity
        state variables for this body,
        joint damping value,
        joint friction coefficient,
        joint lower limit,
        joint upper limit,
        maximum force joint can sustain,
        maximum velocity joint can sustain,
        name string of associated link.
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_joint_state(self, uid, jid):
        """
        Get the current state of body joint
        :param uid: unique integer body id
        :param jid: integer joint id on body
        :return: a tuple of
        joint position float (radian),
        joint velocity float (rad/s),
        wrench on joint (vec6 float, force3 + torque3),
        applied motor torque on joint float.
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_joint_state(self, uid, jids, vals, ctype, kwargs):
        """
        Set the body joint state
        :param uid: integer body unique id
        :param jids: list of joint indices on body
        :param vals: list of values to set on joints.
        Length must align with that of joint indices.
        :param ctype: string control type, choose among
        'position', 'velocity', and 'torque'
        :param kwargs: other key word arguments to the function,
        such as 'reset', 'positionGain', 'velocityGain', 'maxForce', etc.
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def disable_body_joint_motors(self, uid, jids):
        """
        Disable the motors. This is desired for the case
        of direct torque control, where motors need to be
        disabled.
        :param uid: body unique id of the arm
        :param jids: indices of joints to be disabled
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def enable_body_joint_motors(self, uid, jids, forces):
        """
        Enable the motors. This is desired after using
        direct torque control, where motors need to be
        re-activated for other modes of control.
        :param uid: body unique id of the arm
        :param jids: indices of joints to be enabled
        :param forces: force arrays to set all joints
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_dynamics(self, uid, lid):
        """
        Get the body dynamics info
        :param uid: integer body unique id
        :param lid: integer link id on body
        :return: a tuple of
        mass, float kg
        friction coefficient, float
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_dynamics(self, uid, lid, info):
        """
        Change dynamics of body based on given info
        :param uid: integer body unique id
        :param lid: integer link index of body
        :param info: a dictionary, where keys are
        'mass', 'lateral_friction', 'spinning_friction',
        'rolling_friction', or 'restitution' and
        values are corresponding values (float).
        :return: 0 if success, -1 for failure
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_bounding_box(self, uid, lid):
        """
        Get the bounding box of given body
        :param uid: integer body unique id
        :param lid: integer link index of body
        :return: axis-aligned-bounding-box of diagonal
        minimum and maximum coordinates in vec3 float
        cartesian w.r.t. world frame.
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_contact_info(self, uid, lid):
        """
        Get the contacts information on given body link
        :param uid: integer body unique id
        :param lid: integer link index of body
        :return: a list of dictionaries of all
        contact bodies, where dictionary is
        uid=integer unique id of another body,
        lid=integer link index of another body,
        posA=vec3 float cartesian world frame contact position
        on given body,
        posB=vec3 float cartesian world frame contact position
        on another body,
        normalB2A=contact normal force on another body pointing to
        given body,
        distance=contact distance float,
        force=normal force applied.
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_surroundings(self, uidA, lidA, uidB, lidB, dist):
        """
        Get the neighbors of given body and link
        :param uidA: integer body A unique id
        :param lidA: integer link index of body A
        :param uidB: integer body B unique id
        :param lidB: integer link index of body B
        :param dist: examination distance threshold, bodies farther
        than this distance will not be examined
        :return: a list of dictionaries of all nearby bodies,
        where the dictionary is
        uid=neighbor body unique id integer,
        lid=neighbor body link index integer,
        posA=vec3 float cartesian world frame closest point
        on given body,
        posB=vec3 float cartesian world frame closest point
        on another body,
        distance=closest distance between two neighboring bodies
        """
        return NotImplemented

    @abc.abstractmethod
    def add_body_line_marker(self, posA, posB, color, width,
                             time, uid, lid):
        """
        Mark line on given body
        :param posA: vec3 float cartesian world frame position of
        one end of line segment
        :param posB:vec3 float cartesian world frame position of
        the other end of line segment
        :param color: vec3 float RGB in [0,1]
        :param width: float width of line
        :param time: float life time of line in seconds
        :param uid: integer body unique id
        :param lid: integer link index of body
        :return: integer line mark id
        """
        return NotImplemented

    @abc.abstractmethod
    def add_body_text_marker(self, text, pos, font_size, color,
                             uid, lid, time):
        """
        Mark text string on given body
        :param text: text string to be marked.
        :param pos: vec3 float cartesian world frame position
        to add the text
        :param font_size: float of text size
        :param color: vec3 float RGB in [0,1]
        :param uid: integer body unique id
        :param lid: integer link index of body
        :param time: float of text life time, 0 for permanent text
        :return: integer text mark id
        """
        return NotImplemented

    @abc.abstractmethod
    def remove_body_text_marker(self, marker_id):
        """
        Remove the text by given id
        :param marker_id: integer mark id
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def apply_force_to_body(self, uid, lid, force, pos, ref):
        """
        Apply external force to given body/link.
        :param uid: integer body unique id
        :param lid: integer link index of body
        :param force: vec3 float force vector
        :param pos: vec3 float position cartesian
        :param ref: string 'abs' for world frame,
        'rel' for link frame
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def apply_torque_to_body(self, uid, lid, torque, ref):
        """
        Apply external torque to given body/link.
        :param uid: integer body unique id
        :param lid: integer link index of body
        :param torque: vec3 float torque vector
        :param ref: string 'abs' for world frame,
        'rel' for link frame
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def get_body_attachment(self, uid):
        """
        Get the attached bodies on given body
        :param uid: integer body unique id
        :return: a list of dictionaries of dictionaries,
        [{(attached body uid, constraint id): constraint info},...],
        where the constraint info dictionary is
        parentJointIdx=integer index link of given body,
        childLinkIndex=integer link index of attached body,
        type=constraint joint type,
        jointAxis=vec3 float of joint axis in child link frame,
        parentJointPvt=vec3 float position of joint frame relative
        to given body CoM,
        childJointPvt=vec3 float position of joint frame relative
        to attached body CoM,
        parentJointOrn=vec4 float quaternion orientation of joint
        frame relative to given body CoM frame,
        childJointOrn=vec4 float quaternion orientation of joint
        frame relative to attached body CoM frame,
        maxForce=maximum force applied to this constraint)
        """
        return NotImplemented

    @abc.abstractmethod
    def set_body_attachment(self, parent_uid, parent_lid,
                            child_uid, child_lid,
                            jtype='fixed',
                            jaxis=(0., 0., 0.),
                            parent_pos=(0., 0., 0.),
                            child_pos=(0., 0., 0.),
                            **kwargs):
        """
        Attach given child body to given parent body
        based on given parameters
        :param parent_uid: integer unique id of parent body
        :param parent_lid: integer link index of parent body
        :param child_uid: integer unique id of child body
        :param child_lid: integer link index of child body
        :param jtype: joint type, same as listed in
        <get_body_joint_info>
        :param jaxis: vec3 float joint axis in child link frame
        :param parent_pos: vec3 float position of joint frame relative
        to parent body CoM
        :param child_pos: vec3 float position of joint frame relative
        to child body CoM,
        :param kwargs: other keyword arguments, such as
        parentFrameOrientation, childFrameOrientation.
        :return: integer constraint id
        """
        return NotImplemented

    @abc.abstractmethod
    def remove_body_attachment(self, cid):
        """
        Remove the given attachment
        :param cid: constraint id
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def move_body(self, cid, pos, orn, force):
        """
        Move a fixed body to given pose with given force
        :param cid: constraint id on fixed body
        :param pos: new position vec3 float cartesian
        :param orn: new orientation vec4 float quaternion
        :param force: maximum force to move, float
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def delete_body(self, uid):
        """
        Remove given body from environment.
        :param uid: integer body unique id
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def start_engine(self, frame):
        """
        Start the render
        :param frame: string of frame type,
        :return: 0 if success, -1 if failed
        """
        return NotImplemented

    @abc.abstractmethod
    def hold(self, max_steps=1000):
        """
        Leave the world alone for given
        period of time
        :param max_steps: integer number of steps
        to pause external actions in world
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def step(self, elapsed_time, step_size):
        """
        Simulate for one step
        :param elapsed_time: integer elapsed time
        since controller started running, used for
        time out in real time simulation
        :param step_size: dynamically change
        step size during run time
        :return: None
        """
        return NotImplemented

    @abc.abstractmethod
    def stop(self):
        """
        Stop the render and clean up.
        :return: None
        """
        return NotImplemented
