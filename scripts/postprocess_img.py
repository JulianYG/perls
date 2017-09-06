from __future__ import print_function

import os
from glob import glob
import numpy as np

from perls import postprocess

if __name__ == "__main__":

    pp = postprocess.Postprocess('pose', '../configs/gym-cmd.xml', dim='rgbd')

    all_images = list()
    all_states = list()
    all_actions = list()

    record_path = "../src/log/trajectory/push/success/*.bin"

    # Keep files sorted in order to align with random seeds
    files = filter(os.path.isfile, glob(record_path))
    files.sort(key=lambda x: os.path.getmtime(x))

    goals = []
    with open('../src/log/push_sawyer_1.txt', 'r') as f:
        pos_data = f.readlines()
    
    goals = [[float(i) for i in x.split()] for x in pos_data]
    print(goals)

    for i in range(len(files)):
        imgs, states, actions = pp.parse_demonstration(files[i], goals[i])
        all_images.append(imgs)
        all_states.append(states)
        all_actions.append(actions)
    all_images = np.concatenate(all_images, axis=0)
    all_states = np.concatenate(all_states, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    print(all_images.shape)
    print(all_states.shape)
    print(all_actions.shape)
    np.savez("high.npz", imgs=all_images, auxs=all_states, actions=all_actions)
