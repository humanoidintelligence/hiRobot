"""Pose helpers: round trips and composition against hand-worked cases."""

import numpy as np

from hirobot.transforms import (
    IDENTITY_POSE,
    axis_angle_to_quat,
    matrix_to_quat,
    pose_compose,
    pose_from_axis_angle,
    pose_from_matrix,
    pose_inverse,
    pose_to_axis_angle,
    pose_to_matrix,
    quat_to_axis_angle,
    quat_to_matrix,
    rotate,
)

YAW90 = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])  # 90 deg about z


def _same_rotation(a, b):
    return np.allclose(a, b, atol=1e-9) or np.allclose(a, -b, atol=1e-9)


def test_rotate_and_matrix():
    assert np.allclose(rotate(YAW90, [1, 0, 0]), [0, 1, 0])
    assert np.allclose(quat_to_matrix(YAW90) @ [0, 1, 0], [-1, 0, 0])
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        assert _same_rotation(matrix_to_quat(quat_to_matrix(q)), q)


def test_axis_angle_round_trip_including_near_pi():
    for v in ([0, 0, 0], [0.1, -0.2, 0.3], [0.0, 3.14, 0.0], [np.pi - 1e-6, 0, 0]):
        q = axis_angle_to_quat(v)
        assert _same_rotation(axis_angle_to_quat(quat_to_axis_angle(q)), q)
    assert np.allclose(axis_angle_to_quat([0, 0, np.pi / 2]), YAW90)


def test_pose_conversions():
    pose = np.concatenate([[0.1, 0.2, 0.3], YAW90])
    assert np.allclose(pose_from_matrix(pose_to_matrix(pose)), pose)
    assert np.allclose(pose_from_axis_angle(pose_to_axis_angle(pose)), pose)


def test_compose_and_inverse():
    base = np.concatenate([[1.0, 0.0, 0.0], YAW90])          # base 1 m along x, turned 90 deg
    local = np.concatenate([[0.5, 0.0, 0.0], [0, 0, 0, 1]])  # 0.5 m ahead of the base
    world = pose_compose(base, local)
    assert np.allclose(world[:3], [1.0, 0.5, 0.0])
    assert _same_rotation(world[3:], YAW90)
    assert np.allclose(pose_compose(pose_inverse(base), world), local)
    assert np.allclose(pose_compose(base, pose_inverse(base)), IDENTITY_POSE)
