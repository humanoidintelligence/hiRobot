"""Pose math for the 7-vector poses hiRobot uses: ``[x, y, z, qx, qy, qz, qw]``.

Quaternions are ``[x, y, z, w]`` (scalar last) everywhere, the same as
``Observation.base_orientation``. Drivers convert their SDK's form (a 4x4
matrix, axis-angle) with these helpers so every ``ee_poses`` / ``ee_targets``
entry has one layout.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def quat_normalize(q) -> np.ndarray:
    """``q`` scaled to unit length (``[0, 0, 0, 1]`` for a zero quaternion)."""
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return np.array([0.0, 0.0, 0.0, 1.0]) if n == 0 else q / n


def quat_multiply(a, b) -> np.ndarray:
    """Hamilton product ``a * b`` (apply ``b`` first, then ``a``)."""
    ax, ay, az, aw = np.asarray(a, dtype=float)
    bx, by, bz, bw = np.asarray(b, dtype=float)
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conjugate(q) -> np.ndarray:
    """The inverse rotation of a unit quaternion."""
    x, y, z, w = np.asarray(q, dtype=float)
    return np.array([-x, -y, -z, w])


def quat_to_matrix(q) -> np.ndarray:
    """3x3 rotation matrix of a quaternion."""
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m) -> np.ndarray:
    """Quaternion of a 3x3 rotation matrix, with ``w >= 0``."""
    m = np.asarray(m, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    q = quat_normalize(q)
    return -q if q[3] < 0 else q


def axis_angle_to_quat(v) -> np.ndarray:
    """Quaternion of a rotation vector (axis times angle, rad)."""
    v = np.asarray(v, dtype=float)
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = v / angle
    return np.concatenate([axis * np.sin(angle / 2), [np.cos(angle / 2)]])


def quat_to_axis_angle(q) -> np.ndarray:
    """Rotation vector (axis times angle, rad, angle in ``[0, pi]``) of a quaternion."""
    q = quat_normalize(q)
    if q[3] < 0:
        q = -q
    s = np.linalg.norm(q[:3])
    if s < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(s, q[3])
    return q[:3] / s * angle


def rotate(q, v) -> np.ndarray:
    """Vector ``v`` rotated by quaternion ``q``."""
    return quat_to_matrix(q) @ np.asarray(v, dtype=float)


def pose_from_matrix(t) -> np.ndarray:
    """7-vector pose of a 4x4 homogeneous transform."""
    t = np.asarray(t, dtype=float)
    return np.concatenate([t[:3, 3], matrix_to_quat(t[:3, :3])])


def pose_to_matrix(pose) -> np.ndarray:
    """4x4 homogeneous transform of a 7-vector pose."""
    pose = np.asarray(pose, dtype=float)
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(pose[3:7])
    t[:3, 3] = pose[:3]
    return t


def pose_from_axis_angle(p) -> np.ndarray:
    """7-vector pose of ``[x, y, z, rx, ry, rz]`` (position plus rotation vector)."""
    p = np.asarray(p, dtype=float)
    return np.concatenate([p[:3], axis_angle_to_quat(p[3:6])])


def pose_to_axis_angle(pose) -> np.ndarray:
    """``[x, y, z, rx, ry, rz]`` of a 7-vector pose."""
    pose = np.asarray(pose, dtype=float)
    return np.concatenate([pose[:3], quat_to_axis_angle(pose[3:7])])


def pose_compose(a, b) -> np.ndarray:
    """``a ∘ b``: pose ``b`` given in frame ``a``, expressed in ``a``'s parent frame."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.concatenate([a[:3] + rotate(a[3:7], b[:3]),
                           quat_normalize(quat_multiply(a[3:7], b[3:7]))])


def pose_inverse(a) -> np.ndarray:
    """The inverse of pose ``a`` (``pose_compose(a, pose_inverse(a))`` is identity)."""
    a = np.asarray(a, dtype=float)
    qi = quat_conjugate(quat_normalize(a[3:7]))
    return np.concatenate([-rotate(qi, a[:3]), qi])


def euler_xyz_to_quat(rpy: Sequence[float]) -> np.ndarray:
    """``[qx, qy, qz, qw]`` of extrinsic ``xyz`` Euler angles (scipy's ``"xyz"``: ``R = Rz(c) Ry(b) Rx(a)``)."""
    a, b, c = (float(v) for v in rpy)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    # R = Rz(c) @ Ry(b) @ Rx(a)
    m = np.array([
        [cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
        [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
        [-sb, cb * sa, cb * ca],
    ])
    return matrix_to_quat(m)


def quat_to_euler_xyz(quat: Sequence[float]) -> np.ndarray:
    """Extrinsic ``xyz`` Euler angles of ``[qx, qy, qz, qw]`` (inverse of :func:`euler_xyz_to_quat`)."""
    m = quat_to_matrix(quat)
    b = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(m[2, 0]) < 1.0 - 1e-9:
        a = math.atan2(m[2, 1], m[2, 2])
        c = math.atan2(m[1, 0], m[0, 0])
    else:  # gimbal lock: roll and yaw share one axis; put it all in yaw
        a = 0.0
        c = math.atan2(-m[0, 1], m[1, 1])
    return np.array([a, b, c])


IDENTITY_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
