import pybullet as p
import numpy as np
import math

pi = np.pi
_EPS = np.finfo(float).eps * 4.

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


def rms(vector):
    """
    Get the root mean square of the vector
    :param vector: input vector
    :return: float number
    """
    return np.sqrt(np.sum(vector ** 2))


def joint_clip(joint_pos, joint_spec):
    """
    Clip the joints under their limits
    :param joint_pos: input joint positions
    :param joint_spec: upper and lower limit specifications
    :return: Clipped joint position values
    """
    joint_pos = np.clip(
        joint_pos,
        a_min=joint_spec['lower'][-3:],
        a_max=joint_spec['upper'][-3:],
        out=joint_pos)
    return np.arcsin(np.sin(joint_pos))


def approximate(val, n_digits):
    """
    Approximate numbers by given digits.
    :param val: value to approximate.
    :param n_digits: number of digits to keep
    :return: approximated numbers
    """
    return np.array([float('%.{}g'.format(n_digits) % v) for v in val],
                    dtype=np.float32)


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
    homo_pose_mat[3, :3] = np.array(pose[0], dtype=np.float32)
    homo_pose_mat[3, 3] = 1.
    return homo_pose_mat


def relative_pose(obj_pose, frame):
    """
    Get the relative pose of object in given frame,
    in the form of homogeneous matrix.
    :param obj_pose: pose of given object, (pos, orn)
    :param frame: 4x4 homogeneous matrix representing the frame
    :return: pose of the object in the frame, 4x4 homogeneous mat
    """
    obj_pose_homo = pose2mat(obj_pose)
    frame_pose = np.linalg.inv(frame).dot(obj_pose_homo)
    return frame_pose


def transform(poseA, poseB):
    """
    Transform pose into another frame
    :param poseA:
    :param poseB:
    :return:
    """
    return p.multiplyTransforms(
        tuple(poseA[0]), tuple(poseA[1]),
        tuple(poseB[0]), tuple(poseB[1]))


def rand_bigint():
    return np.random.randint(low=500, high=10000)


def vec(values):
    return np.array(values, dtype=np.float32)


def zero_vec(size):
    return np.zeros(size, dtype=np.float32)


def clip_vec(vec, low, high):
    return np.clip(vec, low, high)


def pose_diff(pose1, pose2):

    pos1, orn1 = pose1
    pos2, orn2 = pose2
    pos_xdiff = np.sqrt(np.sum((pos1 - pos2) ** 2))
    orn_xdiff = np.sqrt(np.sum(quat_diff(orn1, orn2) ** 2))
    return pos_xdiff, orn_xdiff


def euler_diff(e1, e2):
    return e1 - e2


def orn_add(quat1, quat2):
    return quat2euler(quat1) + quat2euler(quat2)


def mat4(array):
    return np.array(array, dtype=np.float32).reshape((4, 4))


def mat3(array):
    return np.array(array, dtype=np.float32).reshape((3, 3))


def quat2mat(quaternion):
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


def mat2quat(rmat, isprecise=False):
    M = np.array(rmat, dtype=np.float32, copy=False)[:4, :4]
    if isprecise:
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
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q


def mat2euler(rmat, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = np.array(rmat, dtype=np.float32, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > _EPS:
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


def deg(euler):
    return np.array(euler) * 180. / np.pi


def rad(euler):
    return np.array(euler) * np.pi / 180.


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


def quat_diff(quat1, quat2):
    return quat2euler(quat1) - quat2euler(quat2)


def get_inverse_transformed_pose(body_pose, frame_pose):
    return transform(frame_pose, body_pose)


def get_transformed_pose(body_pose, frame_pose):
    transform_pose = p.invertTransform(frame_pose[0], frame_pose[1])
    return transform(transform_pose, body_pose)


def get_transformed_pos(pos, translation, rotation):
    return rotation.dot(pos - translation)


def process_orn_signal(roll, pitch, yaw, sens):
    orn_euler = np.array([roll, -pitch, yaw],
                         dtype=np.float32) * np.pi / 180.
    return orn_euler * sens


def array2mat():

    # TODO: for graphicsEngine
    pass
