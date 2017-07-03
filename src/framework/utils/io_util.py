import csv
import struct
import os, sys
from xml.etree import ElementTree

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

# Force automatic flush when printing
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)
sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', 0)

_singleton_elem = ElementTree.Element(0)


class FONT:

    # warning for all cases
    warning = ['\033[93m']
    ignore = ['\033[90m']

    # world model color code
    model = ['\033[94m', '\033[95m']

    # display color code
    disp = ['\033[94m', '\033[96m']

    # controller color code
    control = ['\033[92m', '\033[91m']

    end = '\033[0m'
    bold = '\033[1m'
    underline = '\033[4m'


class _EnvTree:
    # TODO: is there any way to generate attr by string
    def __init__(self, env, tool, scene):
        self._env = env
        self._arm, self._gripper = tool
        self._scene_title, self._scene = scene

        #TODO
        self._hand = None

    @property
    def env(self):
        return self._env

    @property
    def arm(self):
        return self._arm

    @property
    def gripper(self):
        return self._gripper

    @property
    def hand(self):
        return self._hand

    @property
    def scene_title(self):
        return self._scene_title

    @property
    def scene(self):
        return self._scene


class _ConfigTree:

    def __init__(self, c_id, build,
                 model_desc, view_desc,
                 name, engine,
                 version, job, video, async,
                 step, max_time, log,
                 record_name):
        self._conf_id = c_id
        self._model_desc = model_desc
        self._view_desc = view_desc
        self._max_run_time = max_time
        self._version = version
        self._job = job
        self._async = async
        self._video = video
        self._name = name
        self._engine = engine
        self._step_size = step
        self._build = build
        self._log_path = log
        self._fname = record_name

    @property
    def build(self):
        return self._build

    @property
    def id(self):
        return self._conf_id

    @property
    def max_run_time(self):
        return self._max_run_time

    @property
    def version(self):
        return self._version

    @property
    def job(self):
        return self._job

    @property
    def video(self):
        return self._video

    @property
    def async(self):
        return self._async

    @property
    def name(self):
        return self._name

    @property
    def engine(self):
        return self._engine

    @property
    def step_size(self):
        return self._step_size

    @property
    def model_desc(self):
        return self._model_desc

    @property
    def view_desc(self):
        return self._view_desc

    @property
    def record_name(self):
        return self._fname

    @property
    def log(self):
        return self._log_path


def str2bool(string):
    return string.lower() == 'true'


def loginfo(msg, itype):
    """
    Print message and flush to terminal
    :param msg: string message to print
    :return: None
    """
    sys.stdout.write('{}{}\n{}'.format(
        itype[0], msg, FONT.end))


def logerr(msg, etype):
    sys.stderr.write('{}{}\n{}'.format(
        etype[1] + FONT.bold, msg, FONT.end))


def parse_log(file, verbose=True):
    f = open(file, 'rb')
    print('Opened'),
    print(file)

    keys = f.readline().decode('utf8').rstrip('\n').split(',')
    fmt = f.readline().decode('utf8').rstrip('\n')

    # The byte number of one record
    sz = struct.calcsize(fmt)
    # The type number of one record
    n_cols = len(fmt)

    if verbose:
        print('Keys:'),
        print(keys)
        print('Format:'),
        print(fmt)
        print('Size:'),
        print(sz)
        print('Columns:'),
        print(n_cols)

    # Read data
    whole_file = f.read()
    # split by alignment word
    chunks = whole_file.split(b'\xaa\xbb')
    log = list()
    for chunk in chunks:
        if len(chunk) == sz:
            values = struct.unpack(fmt, chunk)
            record = list()
            for i in range(n_cols):
                record.append(values[i])
            log.append(record)
    return log


def err(*message):
    with open('error.log', 'a+') as f:
        for msg in message:
            f.write(msg)
            f.write('\n\n')


def parse_env(file_path):
    """
    Parse the given environment description file
    :param file_path: string file path of xml file
    :return: a tuple of elements:
    # TODO
    """
    xml = ElementTree.parse(file_path)
    root = xml.getroot()
    # General environment info
    title = root.attrib['name']
    gravity = float(root.attrib.get('gravity', 1.))
    env = dict(title=title, gravity=gravity)

    gripper_elem = root.findall('./tool/gripper')
    arm_elem = root.findall('./tool/robot')
    scene_elem = root.findall('./scene/body')

    # Tool info
    gripper = parse_gripper_elem(gripper_elem)
    arm = parse_arm_elem(arm_elem)

    # Scene info
    scene = root.find('./scene')
    scene_title = 'Blank'
    if scene is not None:
        scene_title = scene.attrib['name']
        scene = parse_body_elem(scene_elem)
    else:
        scene = list()

    tree = _EnvTree(env, (arm, gripper), (scene_title, scene))
    return tree


def parse_gripper_elem(gripper_elem):
    """
    Parse given gripper xml tree elements.
    :param gripper_elem: a list of gripper ET elements
    :return: a list of dictionaries
    """
    gripper = []
    for i in range(len(gripper_elem)):
        elem = gripper_elem[i]
        asset = elem.find('asset')
        if asset is None:
            asset = _singleton_elem
        pos, orn = elem.find('pos'), elem.find('orn')
        # Grippers are not fixed by default,
        # but you can change them by calling gripper.fix=(orn,pos)
        tid = int(asset.attrib.get('id', i))
        gripper.append(
            dict(
                id=tid,
                # Allow path to be none for gripper (has default)
                path=asset.attrib.get('path', None),
                # Allow none pos/orn to use default as well
                pos=[float(f) for
                     f in pos.text.split(' ')] if pos is not None else None,
                orn=[float(f) for
                     f in orn.text.split(' ')] if orn is not None else None,
                # But must specify type for gripper
                type=elem.attrib['type'],
                # ID refers to controll id
                name='{}_{}'.format(elem.attrib['name'], tid),
                fixed=str2bool(elem.attrib.get('fixed', 'False')),
                traction=float(elem.attrib.get('traction', 200.))
            ))
    return gripper


def parse_arm_elem(arm_elem):
    """
    Parse given arm xml tree elements.
    :param gripper_elem: a list of arm ET elements
    :return: a list of dictionaries
    """
    arm = []
    # Arms are fixed by default, but you can unfix
    # them by calling del arm.fix, this may result in
    # unpredictable behavior!
    for i in range(len(arm_elem)):
        elem = arm_elem[i]
        asset = elem.find('asset')
        if asset is None:
            asset = _singleton_elem
        pos, orn = elem.find('pos'), elem.find('orn')
        # Arm must have at least one gripper.
        gripper_elem = elem.find('gripper')
        tid = int(asset.attrib.get('id', i))
        arm.append(
            dict(
                id=tid,
                path=asset.attrib.get('path', None),
                pos=[float(f) for
                     f in pos.text.split(' ')] if 
                pos is not None else (0., 0., 0.8),
                orn=[float(f) for
                     f in orn.text.split(' ')] if 
                orn is not None else (0., 0., 0., 1.),
                type=elem.attrib['type'],
                name='{}_{}'.format(elem.attrib['name'], tid),
                null_space=str2bool(elem.attrib.get('null_space', 'False')),
                gripper=parse_gripper_elem([gripper_elem])[0]
            )
        )
    return arm


def parse_body_elem(body_elem):
    env = list()
    # For multiple same objects, must indicate oject IDs
    for i in range(len(body_elem)):
        elem = body_elem[i]
        asset = elem.find('asset')
        pos, orn = elem.find('pos'), elem.find('orn')
        env.append(
            dict(
                path=asset.attrib['path'],
                record=str2bool(elem.attrib.get('record', 'False')),
                pos=[float(f) for   # Allow default pos
                     f in pos.text.split(' ')] if
                pos is not None else (0., 0., 0),
                orn=[float(f) for   # Allow default orn
                     f in orn.text.split(' ')] if
                orn is not None else (0., 0., 0., 1.),
                fixed=str2bool(elem.attrib.get('fixed', 'False')),
                # Default id is 0 since first time for each new object
                name='{}_{}'.format(elem.attrib['name'],
                                    asset.attrib.get('id', 0))
            )
        )
    return env


def parse_disp(file_path):
    xml = ElementTree.parse(file_path)
    root = xml.getroot()

    disp_name = root.attrib['name']
    frame_attrib = root.find('./view/frame').attrib
    option_attrib = root.find('./view/option').attrib
    control_attrib = root.find('./control').attrib

    frame_type = frame_attrib['type']
    frame_info = [frame_type]
    if frame_type == 'gui':
        frame_info += [int(frame_attrib.get('key', 0)),
                       frame_attrib.get('flag', '')]
    elif frame_type == 'vr':
        frame_info.append(int(frame_attrib.get('key', 0)))
    elif frame_type == 'udp':
        frame_info += [frame_attrib.get('ip', 'localhost'),
                       int(frame_attrib.get('port', 1234))]
    elif frame_type == 'tcp':
        frame_info += [frame_attrib.get('ip', '127.0.0.1'),
                       int(frame_attrib.get('port', 6667))]

    control_type = control_attrib['type']
    sensitivity = float(control_attrib.get('sensitivity', 1.))
    rate = int(control_attrib.get('rate', 100))
    options = dict((k, str2bool(v)) for (k, v) in option_attrib.items())

    camera_attrib = root.find('./view/camera').attrib
    camera_info = dict(egocentric=str2bool(camera_attrib.get('ego', 'False')),
                       pitch=float(camera_attrib.get('pitch', -35.)),
                       yaw=float(camera_attrib.get('yaw', 50.)),
                       focus=[float(x) for
                              x in camera_attrib.get('focus', '0 0 0').split(' ')],
                       distance=float(camera_attrib.get('distance', 4)))

    return disp_name, frame_info, camera_info, options, \
        control_type, sensitivity, rate


def parse_config(file_path):
    trees = list()
    xml = ElementTree.parse(file_path)
    root = xml.getroot()

    configs = root.findall('./config')

    for conf in configs:

        build = conf.find('./build').attrib['type']
        model_desc = conf.find('./env').text
        view_desc = conf.find('./disp').text

        config_name = conf.attrib['name']
        conf_id = int(conf.attrib['id'])
        simulation_attrib = conf.find('./build/simulator').attrib
        property_attrib = conf.find('./build/property').attrib

        engine = simulation_attrib.get('engine', 'bullet')
        min_version = simulation_attrib.get('version', '20170101')

        job_attrib = conf.find('./build/job').attrib
        job = job_attrib.get('name', 'run')
        video = str2bool(job_attrib.get('video', 'False'))
        log_path = job_attrib.get('log_path', '')
        record_name = job_attrib.get('filename', '')

        async = str2bool(property_attrib.get('async', 'False'))
        step_size = float(property_attrib.get(
            'step_size', 0.001)) if async else None
        max_run_time = int(float(property_attrib.get(
            'max_run_time', 300)))

        # Append one configuration
        trees.append(_ConfigTree(
            conf_id, build, model_desc, view_desc,
            config_name, engine, min_version, job, video,
            async, step_size, max_run_time, log_path, record_name))

    return trees

