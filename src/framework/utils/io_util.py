import csv
import struct
import os, sys
from xml.etree import ElementTree

singleton_elem = ElementTree.Element(0)


class EnvTree(object):

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


def str2bool(string):
    return string.lower() == 'true' if isinstance(string, str) else string

  
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
    scene_title = root.find('./scene').attrib['name']
    scene = parse_body_elem(scene_elem)

    tree = EnvTree(env, (arm, gripper), (scene_title, scene))
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
            asset = singleton_elem
        pos, orn = elem.find('pos'), elem.find('orn')
        # Grippers are not fixed by default,
        # but you can change them by calling gripper.fix=(orn,pos)
        gripper.append(
            dict(# Allow path to be none for gripper (has default)
                path=asset.attrib.get('path', None),
                # Allow none pos/orn to use default as well
                pos=[float(f) for
                     f in pos.text.split(' ')] if pos is not None else None,
                orn=[float(f) for
                     f in orn.text.split(' ')] if orn is not None else None,
                # But must specify type for gripper
                type=elem.attrib['type'],
                # ID refers to controll id
                name='{}_{}'.format(elem.attrib['name'],
                                    asset.attrib.get('id', i))))
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
            asset = singleton_elem
        pos, orn = elem.find('pos'), elem.find('orn')
        # Arm must have at least one gripper.
        gripper_elem = elem.find('gripper')
        arm.append(
            dict(
                path=asset.attrib.get('path', None),
                pos=[float(f) for
                     f in pos.text.split(' ')] if pos is not None else None,
                orn=[float(f) for
                     f in orn.text.split(' ')] if orn is not None else None,
                type=elem.attrib['type'],
                name='{}_{}'.format(elem.attrib['name'],
                                    asset.attrib.get('id', i)),
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
                record=str2bool(elem.attrib.get('record', False)),
                pos=[float(f) for   # Allow default pos
                     f in pos.text.split(' ')] if pos is not None else None,
                orn=[float(f) for   # Allow default orn
                     f in orn.text.split(' ')] if orn is not None else None,
                fixed=str2bool(elem.attrib.get('fixed', False)),
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
    frame_elem = root.find('./view/frame')
    option_elem = root.find('./view/option')

    frame_type = frame_elem.attrib['type']
    frame_info = [frame_type]
    if frame_type == 'gui':
        frame_info += [int(frame_elem.attrib.get('key', 0)),
                       frame_elem.attrib.get('flag', '')]
    elif frame_type == 'vr':
        frame_info.append(int(frame_elem.attrib.get('key', 0)))
    elif frame_type == 'udp':
        frame_info += [frame_elem.attrib.get('ip', 'localhost'),
                       int(frame_elem.attrib.get('port', 1234))]
    elif frame_type == 'tcp':
        frame_info += [frame_elem.attrib.get('ip', '127.0.0.1'),
                       int(frame_elem.attrib.get('port', 6667))]

    control_type = root.find('./control').attrib['type']

    return disp_name, frame_info, option_elem.attrib, control_type


def parse_config(file_path):
    pass

# print parse_env('../../env.xml')

