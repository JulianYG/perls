import pybullet as p
import numpy as np
import math

"""
Note: for all quaternions, uses [x,y,z,w]
"""

pi = np.pi
EPS = np.finfo(float).eps * 4.

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

"""
Scalar Calculation / Processing
"""


def rand_bigint():
    """
    Generate a random big integer
    :return: int
    """
    return np.random.randint(low=500, high=10000)


def fmod(val, mod):
    """
    Floating number mod
    :param val: the float dividend
    :param mod: the float divisor
    :return: remainder float
    """
    return np.fmod(val, mod)


"""
Vector Calculation / Processing
"""


def rms(vector):
    """
    Get the root mean square of the vector
    :param vector: input vector
    :return: float number
    """
    return np.sqrt(np.sum(vector ** 2))


def sign(vector):
    """
    Returns element-wise indication of the sign
    of a vector
    :param vector: input vector
    :return: sign of each element in the vector
    """
    return np.sign(vector)


def approximate(val, n_digits):
    """
    Approximate numbers by given digits.
    :param val: value to approximate.
    :param n_digits: number of digits to keep
    :return: approximated numbers
    """
    return np.around(val, decimals=n_digits)


def cross(x, y, axis=None):
    """
    Perform cross product of values.
    :param x: vector X.
    :param y: vector Y.
    :param axis: axis to cross on
    :return: the cross product of x, y
    """
    if axis:
        return np.cross(x, y, axis=axis)
    return np.cross(x, y)


def vec(values):
    """
    Convert value tuple into a vector
    :param values: a tuple of numbers
    :return: vector of given values
    """
    return np.array(values, dtype=np.float32)


def zero_vec(size):
    """
    Generate a vector of zeros
    :param size: desired size of the vector
    :return: zero vector with given size
    """
    return np.zeros(size, dtype=np.float32)


def clip_vec(vec, low, high):
    """
    Clip the vector
    :param vec: vector to be clipped
    :param low: minimum value(s) in vector
    :param high: maximum value(s) in vector
    :return: clipped vector
    """
    return np.clip(vec, low, high)


def pose_diff(pose1, pose2):
    """
    Get the RMS difference between poses
    :param pose1: (pos, orn)
    :param pose2: (pos, orn)
    :return: RMS difference in (pos, orn)
    """
    pos1, orn1 = pose1
    pos2, orn2 = pose2
    pos_xdiff = np.sqrt(np.sum((pos1 - pos2) ** 2))
    orn_xdiff = np.sqrt(np.sum(quat_diff(orn1, orn2) ** 2))
    return pos_xdiff, orn_xdiff


def pos_diff(pos1, pos2, axis=0, weights=None):
    """
    Find the absolute squared difference between given positions
    :param pos1: numbers in some vectorizable form
    :param pos2: numbers in some vectorizable form
    :return: deltas between pos1 and pos2, as in
    sum((pos1 - pos2) ** 2) along axis
    """
    pos1 = np.array(pos1, dtype=np.float32)
    pos2 = np.array(pos2, dtype=np.float32)
    if weights is None:
        delta = np.sum((pos1 - pos2) ** 2, axis=axis)
    else:
        delta = np.sum(((pos1 - pos2) * weights) ** 2, axis=axis)
    return np.sqrt(delta)


"""
Matrix Calculation / processing
"""


def mat4(array):
    """
    Convert an array to 4x4 matrix
    :param array: the array in form of vec, list, or tuple
    :return: 4x4 numpy matrix
    """
    return np.array(array, dtype=np.float32).reshape((4, 4))


def mat3(array):
    """
    Convert an array to 3x3 matrix
    :param array: the array in form of vec, list, or tuple
    :return: 3x3 numpy matrix
    """
    return np.array(array, dtype=np.float32).reshape((3, 3))


def mat_inv(matrix):
    """
    Calculate the inverse of a matrix
    :param matrix: matrix
    :return: inverse matrix
    """
    return np.linalg.inv(matrix)


"""
Angle vector Calculation / Processing
"""


def deg(euler):
    """
    Convert radian angle to degrees
    :param euler: angle in radian
    :return: angle in degree
    """
    return np.array(euler) * 180. / np.pi


def rad(euler):
    """
    Convert degree angle to radian
    :param euler: angle in degree
    :return: angle in radian
    """
    return np.array(euler) * np.pi / 180.


def euler_diff(e1, e2):
    """
    Get the L1 difference between euler angles
    :param e1: vec3 float deg/rad
    :param e2: vec3 float deg/rad
    :return: difference between given angles
    """
    return e1 - e2


def quat_diff(quat1, quat2):
    """
    Calculate the difference between given quaternions
    :param quat1: The minuend vec4 float
    :param quat2: The subtrahend vec4 float
    :return: The difference between given quaternions in
    radians vec3 float.
    """
    return quat2euler(quat1) - quat2euler(quat2)


def quat_sum(quat1, quat2):
    """
    Add two quaternions
    :param quat1: vec4 float orientation
    :param quat2: vec4 float orientation
    :return: vec4 float orientation sum
    """
    return euler2quat(quat2euler(quat1) + quat2euler(quat2))


def quat_inv(quaternion):
    """
    Calculate the inverse of a quaternion
    :param quaternion: vec4 float orientation quaternion
    :return: the inverse of the given quaternion
    """
    q = np.array(quaternion, dtype=np.float32, copy=True)
    np.negative(q[1:], q[1:])
    return q / np.dot(q, q)


def quat_mul(quaternion0, quaternion1):
    """
    Calculate the product of two quaternions
    :param quaternion0: vec4 float
    :param quaternion1: vec4 float
    :return: The product of two quaternions
    """
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float32)


def quat2euler(quaternion):
    """
    Convert quaternion to euler angle in radian
    :param quaternion: vec4 float angles in quaternion
    :return: vec3 float angles in radian
    """
    return np.array(p.getEulerFromQuaternion(quaternion))


def euler2quat(euler):
    """
    Convert euler angle in radian to quaternion
    :param euler: vec3 float angles in radian
    :return: vec4 float angles in quaternion
    """
    return np.array(p.getQuaternionFromEuler(euler))
    

"""
Matrix pose / rotation calculation / processing
"""


def pose2mat(pose):
    """
    Convert pose to homogeneous matrix
    :param pose: a (pos, orn) tuple where
    pos is vec3 float cartesian, and
    orn is vec4 float quaternion.
    :return:
    """
    homo_pose_mat = np.zeros((4, 4), dtype=np.float32)
    homo_pose_mat[:3, :3] = quat2mat(pose[1])
    homo_pose_mat[:3, 3] = np.array(pose[0], dtype=np.float32)
    homo_pose_mat[3, 3] = 1.
    return homo_pose_mat


def mat2pose(hmat):
    """
    Convert a homogeneous 4x4 matrix into pose
    :param hmat: a 4x4 homogeneous matrix
    :return: (pos, orn) tuple where pos is 
    vec3 float in cartesian, orn is vec4 float quaternion
    """
    pos = hmat[:3, 3]
    orn = mat2quat(hmat[:3, :3])
    return pos, orn


def quat2mat(quaternion):
    """
    Convert given quaternion to matrix
    :param quaternion: vec4 float angles
    :return: 3x3 rotation matrix
    """
    q = np.array(quaternion, dtype=np.float32, copy=True)[[3,0,1,2]]
    n = np.dot(q, q)
    if n < EPS:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]],
        [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]],
        [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]]])


def mat2quat(rmat, precise=False):
    """
    Convert given rotation matrix to quaternion
    :param rmat: 3x3 rotation matrix
    :param precise: If isprecise is True,
    the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.
    :return: vec4 float quaternion angles
    """
    M = np.array(rmat, dtype=np.float32, copy=False)[:3, :3]
    if precise:
        q = np.empty((4,))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00 - m11 - m22, 0.0, 0.0, 0.0],
                      [m01 + m10, m11 - m00 - m22, 0.0, 0.0],
                      [m02 + m20, m12 + m21, m22 - m00 - m11, 0.0],
                      [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22]])
        K /= 3.0
        # quaternion is Eigen vector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q[[1,2,3,0]]


def mat2euler(rmat, axes='sxyz'):
    """
    Convert given rotation matrix to euler angles in radian.
    :param rmat: 3x3 rotation matrix
    :param axes: One of 24 axis sequences as string or encoded tuple
    :return: converted euler angles in radian vec3 float
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = np.array(rmat, dtype=np.float32, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return vec((ax, ay, az))


def euler2mat(euler, axes='sxyz'):
    """
    Convert given euler angle to rotation matrix
    :param euler: vec3 float euler angles in radian
    :param axes: one of 24 axis sequences as string or encoded tuple
    :return: 3x3 rotation matrix
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    ai, aj, ak = euler

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj * si
        M[i, k] = sj * ci
        M[j, i] = sj * sk
        M[j, j] = -cj * ss + cc
        M[j, k] = -cj * cs - sc
        M[k, i] = -sj * ck
        M[k, j] = cj * sc + cs
        M[k, k] = cj * cc - ss
    else:
        M[i, i] = cj * ck
        M[i, j] = sj * sc - cs
        M[i, k] = sj * cc + ss
        M[j, i] = cj * sk
        M[j, j] = sj * ss + cc
        M[j, k] = sj * cs - sc
        M[k, i] = -sj
        M[k, j] = cj * si
        M[k, k] = cj * ci
    return M[:3, :3]


"""
Transformations
"""


def transform(body_pose, transform_pose):
    """
    Transform given pose by another given transformation pose
    :param body_pose: pose (pos, orn) to be transformed
    :param transform_pose: the transformation matrix in form of pose tuple
    :return: The transformed poseA (pos, orn) in its current frame
    """
    return p.multiplyTransforms(
        tuple(body_pose[0]), tuple(body_pose[1]),
        tuple(transform_pose[0]), tuple(transform_pose[1]))


def get_absolute_pose(body_pose, frame_pose):
    """
    Given world pose of a reference frame, and
    frame pose of an object in that frame, calculate
    the pose of the object in the world frame.
    :param body_pose: pose (pos, orn) of body in the given frame
    :param frame_pose: frame (pos, orn) in absolute world frame
    :return: absolute pose of object (pos, orn)
    """
    return transform(frame_pose, body_pose)


def get_relative_pose(body_pose, frame_pose):
    """
    Get the relative pose of body in given frame.
    :param body_pose: pose of given object, (pos, orn)
    :param frame_pose: pose of the reference frame, (pos, orn)
    :return: pose of the object in the frame, (pos, orn)
    """
    # Invert transformation first
    transform_pose = p.invertTransform(frame_pose[0], frame_pose[1])
    # T^-1 x P
    mat = pose2mat(transform(transform_pose, body_pose))
    return mat[:3, 3], mat2quat(mat[:3, :3])
