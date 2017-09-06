from __future__ import print_function

import os
from glob import glob
import numpy as np

from perls import postprocess

if __name__ == "__main__":

    pp = postprocess.Postprocess('pose', '../configs/gym-cmd.xml', dim='rgbd')

    all_states = list()
    all_actions = list()

    record_path = "../src/log/trajectory/push/success/*.bin"

    # Keep files sorted in order to align with random seeds
    files = filter(os.path.isfile, glob(record_path))
    files.sort(key=lambda x: os.path.getmtime(x))

    np.random.seed(42)
    for fname in files:
        states, actions = pp.parse_demonstration(fname)
        all_states.append(states)
        all_actions.append(actions)
    all_states = np.concatenate(all_states, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    print(all_states.shape)
    print(all_actions.shape)
    np.savez("high.npz", states=all_states, actions=all_actions)
