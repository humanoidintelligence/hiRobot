"""Camera intrinsics and mounts: ``CameraIntrinsics``, the ``intrinsics:`` config key, ``Robot.camera_info``."""

from __future__ import annotations

import math

import numpy as np
import pytest

from hirobot import make_robot
from hirobot.core.embodiment import CameraIntrinsics
from hirobot.devices.camera import CameraDriver

CAM = {"device": "camera", "width": 64, "height": 48, "fovy": 50, "driver": {"type": "array"}}


def _robot(**cam):
    """A toy arm with one ``scene`` camera fixed 1 m above the node origin."""
    return make_robot({"assembly": {"name": "cell", "parts": {
        "body": {"from": "body/ur/ur5e", "overrides": {"driver": {"type": "toy"}}},
        "scene": {**CAM, **cam}},
        "placement": {"scene": {"pose": [0, 0, 1, 0, 0, 0, 1]}}}})


# ------------------------------------------------------------- CameraIntrinsics


def test_K_and_projection_convention():
    intr = CameraIntrinsics(width=640, height=480, fx=600, fy=610, cx=320.5, cy=240.25)
    assert np.array_equal(intr.K, [[600, 0, 320.5], [0, 610, 240.25], [0, 0, 1]])
    # A point 1 m ahead, 0.1 m right and 0.05 m down lands right of and below the centre.
    u, v, w = intr.K @ np.array([0.1, 0.05, 1.0])
    assert (u / w, v / w) == pytest.approx((380.5, 270.75))


def test_from_fovy_matches_a_pinhole():
    intr = CameraIntrinsics.from_fovy(640, 480, 60.0)
    assert intr.fx == intr.fy == pytest.approx(240 / math.tan(math.radians(30)))
    assert (intr.cx, intr.cy) == (319.5, 239.5)  # centre of a pixel grid starting at 0
    assert intr.model == "none" and intr.coeffs == ()


def test_scaled_keeps_the_field_of_view():
    full = CameraIntrinsics.from_fovy(1280, 960, 45.0)
    half = full.scaled(640, 480)
    assert half == CameraIntrinsics.from_fovy(640, 480, 45.0)


@pytest.mark.parametrize("bad", [
    {"fx": 0},
    {"width": 0},
    {"model": "fisheye_weird"},
    {"model": "brown_conrady", "coeffs": [0.1, 0.2]},
    {"model": "none", "coeffs": [0.1, 0, 0, 0, 0]},
])
def test_rejects_bad_intrinsics(bad):
    with pytest.raises(ValueError):
        CameraIntrinsics(**{"width": 64, "height": 48, "fx": 50, "fy": 50, "cx": 31.5, "cy": 23.5,
                            **bad})


def test_zero_coefficients_mean_no_distortion():
    intr = CameraIntrinsics(width=64, height=48, fx=50, fy=50, cx=31.5, cy=23.5, coeffs=[0, 0, 0])
    assert intr.coeffs == ()


# ------------------------------------------------------------------ camera_info


def test_sim_camera_falls_back_to_fovy():
    robot = _robot()
    info = robot.camera_info("scene")
    assert info.intrinsics_source == "fovy"
    assert info.intrinsics == CameraIntrinsics.from_fovy(64, 48, 50)
    assert info.mount.pos == (0.0, 0.0, 1.0) and info.depth is False


def test_config_intrinsics_win_and_default_to_the_camera_size():
    robot = _robot(intrinsics={"fx": 55.0, "fy": 56.0, "cx": 30.0, "cy": 22.0,
                               "model": "brown_conrady", "coeffs": [0.1, 0, 0, 0, 0]})
    info = robot.camera_info("scene")
    assert info.intrinsics_source == "config"
    assert (info.intrinsics.width, info.intrinsics.height) == (64, 48)
    assert (info.intrinsics.fx, info.intrinsics.cy) == (55.0, 22.0)
    assert info.intrinsics.coeffs == (0.1, 0.0, 0.0, 0.0, 0.0)


def test_config_intrinsics_at_another_size_are_rescaled():
    robot = _robot(intrinsics={"width": 128, "height": 96, "fx": 110.0, "fy": 110.0,
                               "cx": 63.5, "cy": 47.5})
    intr = robot.camera_info("scene").intrinsics
    assert (intr.width, intr.height, intr.fx, intr.cx, intr.cy) == (64, 48, 55.0, 31.5, 23.5)


def test_bad_config_intrinsics_name_the_camera():
    with pytest.raises(ValueError, match="scene.*intrinsics"):
        _robot(intrinsics={"fx": -1, "fy": 1, "cx": 0, "cy": 0})
    with pytest.raises(ValueError, match="intrinsics"):
        _robot(intrinsics=[1, 2, 3, 4])


class _ReportingCamera(CameraDriver):
    """A camera whose SDK reports intrinsics at twice the delivered size."""

    def start(self):
        pass

    def latest(self):
        return None

    def stop(self):
        pass

    def intrinsics(self):
        return CameraIntrinsics(width=128, height=96, fx=100.0, fy=100.0, cx=64.0, cy=48.0)


def test_driver_intrinsics_beat_fovy_and_lose_to_config():
    robot = _robot()
    robot.cameras["scene"] = _ReportingCamera("scene")
    info = robot.camera_info("scene")
    assert info.intrinsics_source == "driver"
    assert (info.intrinsics.width, info.intrinsics.fx) == (64, 50.0)  # rescaled to 64x48
    robot = _robot(intrinsics={"fx": 55.0, "fy": 55.0, "cx": 31.5, "cy": 23.5})
    robot.cameras["scene"] = _ReportingCamera("scene")
    assert robot.camera_info("scene").intrinsics_source == "config"


def test_child_part_camera_reaches_its_driver():
    robot = make_robot({"assembly": {"name": "pair", "parts": {
        "left": {"from": "robots/trossen_wx250s"},
        "scene": CAM}}})
    names = [c.name for c in robot.embodiment.cameras]
    assert "left/wrist" in names
    # The toy body skips the RealSense drivers, so the child camera falls back to fovy.
    info = robot.camera_info("left/wrist")
    assert info.intrinsics_source == "fovy" and info.depth is True
    assert robot._camera_driver("left/wrist") is None
    assert robot._camera_driver("scene") is robot.cameras["scene"]


def test_unknown_camera():
    with pytest.raises(KeyError, match="no camera 'nope'"):
        _robot().camera_info("nope")


def test_camera_without_mount_or_intrinsics():
    robot = make_robot({"assembly": {"name": "cell", "parts": {
        "body": {"from": "body/ur/ur5e", "overrides": {"driver": {"type": "toy"}}}, "scene": CAM}}})
    info = robot.camera_info("scene")
    # Not placed: the camera sits at the node origin, which still gives a mount and fovy.
    assert info.mount is not None and info.intrinsics_source == "fovy"
