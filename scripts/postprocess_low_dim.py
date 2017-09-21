from __future__ import print_function

import os
from glob import glob
import numpy as np

from perls import postprocess

if __name__ == "__main__":

    pp = postprocess.Postprocess('vel', '../perls/configs/gym-cmd.xml', dim='low')

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
    print(goals)

    for i in range(len(files)):
        states, actions = pp.parse_demonstration(files[i], goals[i])
        all_states.append(states)
        all_actions.append(actions)
    all_states = np.concatenate(all_states, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    print(all_states.shape)
    print(all_actions.shape)
    np.savez("velocity.npz", states=all_states, actions=all_actions)
