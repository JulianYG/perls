from __future__ import print_function

import os, errno
import shutil
from glob import glob
import numpy as np

from perls import postprocess, io_util


if __name__ == "__main__":

    pp = postprocess.Postprocess('vel', '../perls/configs/gym-cmd.xml', dim='rgbd', use_display=False)

    all_images = list()
    all_states = list()
    all_actions = list()

    record_path = "../perls/log/trajectory/push/success/*.bin"

    # Keep files sorted in order to align with random seeds
    files = list(filter(os.path.isfile, glob(record_path)))
    files.sort(key=lambda x: os.path.basename(x))

    goals = []
    with open('../perls/log/push_sawyer.txt', 'r') as f:
        pos_data = f.readlines()
    
    goals = [[float(i) for i in x.split()] for x in pos_data]

    try:
        os.mkdir("tmp")
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    for i in range(len(files)):
        imgs, states, actions = pp.parse_demonstration(files[i], goals[i])
        np.savez("tmp/{}.npz".format(i), imgs=imgs, auxs=states, actions=actions)

    npz_files = list(filter(os.path.isfile, glob("tmp/*.npz")))
    npz_files.sort(key=lambda x: os.path.basename(x))
    for f in npz_files:
        x = np.load(f)
        all_images.append(x["imgs"])
        all_states.append(x["auxs"])
        all_actions.append(x["actions"])
    all_images = np.concatenate(all_images, axis=0)
    all_states = np.concatenate(all_states, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    print(all_images.shape)
    print(all_states.shape)
    print(all_actions.shape)

    if pp.name != 'pose' and pp.name != 'vel':
        np.savez("velocity_new.npz", imgs=all_images, auxs=all_states, actions=all_actions)
        shutil.rmtree("tmp", ignore_errors=True)
    else:
        np.savez('{}.npz'.format(pp.name), states=all_states, actions=all_actions)
