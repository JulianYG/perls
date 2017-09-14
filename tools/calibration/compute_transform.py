
import pickle, os
import numpy as np 

from os.path import join as pjoin
from tf.transformations import quaternion_from_matrix as qfm

ROOT = pjoin(os.getcwd(), 'calib_data/kinect')

RGB2ROBOT = np.zeros((4, 4), dtype=np.float32)
RGB2ROBOT[3, 3] = 1.

with open(pjoin(ROOT, 'robot_RGB_rotation.p'), 'rb') as f:
    r = pickle.load(f)
    RGB2ROBOT[:3, :3] = r
    print('RGB static transform quaternion: {}'.format(qfm(RGB2ROBOT.T)))

with open(pjoin(ROOT, 'robot_RGB_translation.p'), 'rb') as f:
    t = pickle.load(f).reshape((3,))
    print('RGB static transform translation: {}'.format(-r.T.dot(t)/ 1000.))

RGB2ROBOT[:3, 3] = t

IR2RGB = np.zeros((4, 4), dtype=np.float32)
IR2RGB[3, 3] = 1

with open(pjoin(ROOT, 'IR_RGB_rotation.p'), 'rb') as f:
    IR2RGB[:3, :3] = pickle.load(f)
with open(pjoin(ROOT, 'IR_RGB_translation.p'), 'rb') as f:
    IR2RGB[:3, 3] = pickle.load(f).reshape((3,))

IR2ROBOT = RGB2ROBOT.dot(IR2RGB)

stereo_r = IR2ROBOT[:3, :3]
stereo_t = IR2ROBOT[:3, 3]

print('IR static transform quaternion: {}'.format(qfm(IR2ROBOT.T)))
print('IR static transform translation: {}'.format(-stereo_r.T.dot(stereo_t) / 1000.))

with open(pjoin(ROOT, 'robot_IR_rotation.p'), 'wb') as f:
    pickle.dump(stereo_r, f)

with open(pjoin(ROOT, 'robot_IR_translation.p'), 'wb') as f:
    pickle.dump(stereo_t, f)
