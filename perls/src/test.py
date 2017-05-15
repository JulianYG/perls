import cv2
import numpy as np

D = np.array([[0.0, 0.0, 0.0, 0.0]])

# Camera matrix
K = np.array([[1, 0.0, 0],
              [0.0, 1, 0],
              [0.0, 0.0, 1.0]])


UV_cp = np.array([[1300.0, 2544.0], # left down
                  [1607.0, 1000.0], # left up
                  [3681.0, 2516.0], # right down
                  [3320.0, 983.0]], np.float32) # right up

# Z is on 0 plane, so Z=0.0
XYZ_gcp = np.array([[0.0, 400.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [300.0, 400.0, 0.0],
                    [300.0, 0.0, 0.0]], np.float32)

retval, rvec, tvec = cv2.solvePnP(XYZ_gcp, UV_cp, K, D)
print rvec