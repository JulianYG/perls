import pybullet as p
import numpy as np
import math

_EPS = np.finfo(float).eps * 4.


def approximate(val, n_digits):
    return np.array([float('%.{}g'.format(n_digits) % v) for v in val],
                    dtype=np.float32)


def vec(values):
    return np.array(values, dtype=np.float32)


def zero_vec(size):
    return np.zeros(size, dtype=np.float32)


def quat2euler(quaternion):
    return np.array(p.getEulerFromQuaternion(quaternion))


def euler2quat(euler):
    return np.array(p.getQuaternionFromEuler(euler))


def mat_inv(matrix):
    return np.linalg.inv(matrix)


def quat_inv(quaternion):
    q = np.array(quaternion, dtype=np.float32, copy=True)
    np.negative(q[1:], q[1:])
    return q / np.dot(q, q)


def quat_mul(quaternion0, quaternion1):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float32)


def get_transformed_pos(pos, translation, rotation):
    return rotation.dot(pos - translation)


def get_rotation_from_quaternion(quaternion):
    q = np.array(quaternion, dtype=np.float32, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]],
        [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]],
        [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]]])

