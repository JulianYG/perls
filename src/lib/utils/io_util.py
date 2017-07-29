import struct
import collections
import numpy as np
import os, sys
from xml.etree import ElementTree

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'

# Force automatic flush when printing
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)
sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', 0)

np.set_printoptions(precision=3, suppress=True)
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

_env_tree = collections.namedtuple(
    'EnvTree',
    ['env', 'arm', 'gripper', 'hand',
     'scene_title', 'scene']
)

_config_tree = collections.namedtuple(
    'ConfigTree',
    ['id', 'build', 'model_desc', 'view_desc',
     'config_name', 'physics_engine', 'graphics_engine',
     'min_version', 'job', 'video',
     'async', 'step_size', 'max_run_time', 'log',
     'record_name', 'control_type', 'sensitivity',
     'rate', 'disp_info'])


def str2bool(string):
    return string.lower() == 'true'


def loginfo(msg, itype):
    """
    Print message and flush to terminal
    :param msg: string message to print
    :return: None
    """
    # msg = pprint.pformat(msg)
    sys.stdout.write('{}{}\n{}'.format(
        itype[0], msg, FONT.end))


def logerr(msg, etype):
    # msg = pprint.pformat(msg)
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

    tree = _env_tree(env, arm, gripper, None, scene_title, scene)
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
                ik_path=asset.attrib.get('ik_path', None),
                pos=[float(f) for
                     f in pos.text.split(' ')] if 
                pos is not None else (0., 0., 0.8),
                orn=[float(f) for
                     f in orn.text.split(' ')] if 
                orn is not None else (0., 0., 0., 1.),
                type=elem.attrib['type'],
                name='{}_{}'.format(elem.attrib['name'], tid),
                collision_checking=str2bool(elem.attrib.get('collision_checking', 'False')),
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
                # Default id is 0 since first time for each new entity
                name='{}_{}'.format(elem.attrib['name'],
                                    asset.attrib.get('id', 0))
            )
        )
    return env


def parse_disp(file_path):
    xml = ElementTree.parse(file_path)
    root = xml.getroot()

    option_attrib = root.find('./view/option').attrib
    options = dict((k, str2bool(v)) for (k, v) in option_attrib.items())

    camera_attrib = root.find('./view/camera').attrib
    camera_info = dict(egocentric=str2bool(camera_attrib.get('ego', 'False')),
                       pitch=float(camera_attrib.get('pitch', -35.)),
                       yaw=float(camera_attrib.get('yaw', 50.)),
                       focus=[float(x) for
                              x in camera_attrib.get('focus', '0 0 0').split(' ')],
                       flen=float(camera_attrib.get('focal_len', 4)))

    return camera_info, options


def parse_config(file_path):
    trees = list()
    xml = ElementTree.parse(file_path)
    root = xml.getroot()

    configs = root.findall('./config')

    for conf in configs:

        build = conf.find('./build').attrib['type'].lower()
        model_desc = conf.find('./env').text
        view_desc = conf.find('./disp').text

        config_name = conf.attrib['name']
        conf_id = int(conf.attrib['id'])

        # Getting five main attributes
        physics_attrib = conf.find('./build/physics').attrib
        graphics_attrib = conf.find('./build/graphics').attrib
        property_attrib = conf.find('./build/property').attrib
        job_attrib = conf.find('./build/job').attrib
        control_attrib = root.find('./config/control').attrib

        physics_engine = physics_attrib.get('render', 'bullet')
        graphics_engine = graphics_attrib.get('render', 'bullet')

        min_version = physics_attrib.get('version', '20170101')

        display_name = graphics_attrib['name']

        display_type = graphics_attrib['type'].lower()
        disp_args = [display_type]
        if display_type == 'gui':
            disp_args += [int(graphics_attrib.get('key', 0)),
                              graphics_attrib.get('flag', '')]
        elif display_type == 'vr':
            disp_args.append(int(graphics_attrib.get('key', 0)))
        elif display_type == 'udp':
            disp_args += [graphics_attrib.get('ip', 'localhost'),
                          int(graphics_attrib.get('port', 1234))]
        elif display_type == 'tcp':
            disp_args += [graphics_attrib.get('ip', '127.0.0.1'),
                          int(graphics_attrib.get('port', 6667))]

        disp_info = (display_name, display_type, disp_args)

        job = job_attrib.get('name', 'run').lower()
        video = str2bool(job_attrib.get('video', 'False'))
        log_path = job_attrib.get('log_path', '')
        record_name = job_attrib.get('filename', '')

        async = str2bool(property_attrib.get('async', 'False'))
        step_size = float(property_attrib.get(
            'step_size', 0.001)) if async else None
        max_run_time = int(float(property_attrib.get(
            'max_run_time', 300)))

        control_type = control_attrib['type'].lower()
        sensitivity = float(control_attrib.get('sensitivity', 1.))
        rate = int(control_attrib.get('rate', 100))

        # Append one configuration
        trees.append(
            _config_tree(
                conf_id, build, model_desc, view_desc,
                config_name, physics_engine, graphics_engine,
                min_version, job, video,
                async, step_size, max_run_time, log_path, record_name,
                control_type, sensitivity, rate,
                disp_info)
        )
    return trees
