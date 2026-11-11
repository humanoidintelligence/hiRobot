"""The shared MuJoCo world (hirobot.sim.sim_world): several bodies in one model.

Uses the UR5e MJCF from MuJoCo Menagerie (universal_robots_ur5e/ur5e.xml) at
$HIROBOT_UR5E_MJCF, default ~/.cache/hirobot/mujoco_menagerie/; skipped when it or
mujoco is missing.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from hirobot import Action, ControlMode, make_robot

mujoco = pytest.importorskip("mujoco")

UR5E = Path(os.environ.get("HIROBOT_UR5E_MJCF", Path.home() / ".cache/hirobot/mujoco_menagerie"
                           "/universal_robots_ur5e/ur5e.xml")).expanduser()
pytestmark = pytest.mark.skipif(not UR5E.is_file(), reason=f"UR5e MJCF not found at {UR5E}")

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
MODEL = {"model": {"mjcf": str(UR5E), "kp": [2000, 2000, 2000, 500, 500, 500],
                   "kd": [100, 100, 100, 20, 20, 20], "home": HOME}}


def _dual(**sim):
    cfg = {"defaults": ["robots/ur5e_dual"],
           "sim": {"realtime": False, "ramp_seconds": 0.2, **sim},
           "assembly": {"parts": {"left": {"overrides": {"body": MODEL}},
                                  "right": {"overrides": {"body": MODEL}}}}}
    return make_robot(cfg)


def _views(robot):
    from hirobot.drivers.sim.mujoco.driver import MujocoView

    return {p: r for p, r in _walk(robot) if isinstance(r, MujocoView)}


def _walk(robot, path=""):
    inner = robot
    while getattr(inner, "_inner", None) is not None and getattr(inner, "_node", None) is None:
        inner = inner._inner
    node = getattr(inner, "_node", None)
    if node is None:
        yield path, getattr(inner, "body", inner)
        return
    for name, child in node.robots.items():
        yield from _walk(child, f"{path}{name}/")
    if getattr(inner, "body", None) is not None:
        yield path, inner.body


def test_two_arms_in_one_world_share_one_clock_and_keep_their_own_joints():
    robot = _dual()
    robot.connect()
    try:
        views = _views(robot)
        assert set(views) == {"left/", "right/"}
        left, right = views["left/"], views["right/"]
        assert left.world is right.world and left.model is right.model
        m = left.model
        # One model, names prefixed with the part path; each view owns its six joints.
        assert m.joint("left/shoulder_pan").id != m.joint("right/shoulder_pan").id
        assert set(left._qadr).isdisjoint(right._qadr)
        # Placement: left base at y = +0.4, right at -0.4.
        np.testing.assert_allclose(left.data.body("left/base").xpos, [0, 0.4, 0], atol=1e-9)
        np.testing.assert_allclose(right.data.body("right/base").xpos, [0, -0.4, 0], atol=1e-9)

        robot.reset()
        t0 = robot.now()
        target = np.array(HOME * 2)
        target[0] = 0.5  # left/shoulder_pan only
        for _ in range(60):
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
        # One world step per command, not one per arm: a step is the 30 Hz period
        # rounded to whole 2 ms physics steps (17, 0.034 s), as for MujocoRobot.
        world = left.world
        step = world.substeps * m.opt.timestep
        assert world.substeps == round((1 / 30) / m.opt.timestep)
        assert robot.now() - t0 == pytest.approx(60 * step, abs=1e-9)
        q = robot.get_observation().joint_pos
        assert q[0] == pytest.approx(0.5, abs=0.02)
        assert q[6] == pytest.approx(0.0, abs=0.02)  # right arm held
        np.testing.assert_allclose(q[:6][1:], HOME[1:], atol=0.03)
    finally:
        robot.disconnect()


def test_an_unmentioned_arm_still_steps_with_the_world():
    robot = _dual()
    robot.connect()
    try:
        robot.reset()
        world = _views(robot)["left/"].world
        t0 = robot.now()
        right = Action(mode=ControlMode.POSITION, joint_target=np.array(HOME))
        for _ in range(15):
            robot.send_action(Action(parts={"right": right}))  # left idles
        assert robot.now() - t0 == pytest.approx(15 * world.substeps * world.model.opt.timestep,
                                                 abs=1e-9)
    finally:
        robot.disconnect()


def test_servo_actuators_become_pd_motors_with_the_force_range_kept():
    robot = _dual()
    robot.connect()
    try:
        left = _views(robot)["left/"]
        m = left.model
        a = left._act[0]
        assert m.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_AFFINE
        assert not m.actuator_ctrllimited[a]
        np.testing.assert_allclose(m.actuator_forcerange[a], [-150, 150])
        np.testing.assert_allclose(m.actuator_forcerange[left._act[5]], [-28, 28])
    finally:
        robot.disconnect()


def test_scene_and_unknown_sim_keys(tmp_path):
    scene = tmp_path / "table.xml"
    scene.write_text('<mujoco><worldbody><geom name="table" type="box" size="0.5 0.5 0.02" '
                     'pos="0.5 0 -0.02"/></worldbody></mujoco>')
    robot = _dual(scene={"mjcf": str(scene)})
    robot.connect()
    try:
        m = _views(robot)["left/"].model
        assert m.geom("table").id >= 0
    finally:
        robot.disconnect()
    with pytest.raises(ValueError, match="not a body of this robot"):
        _dual(fixed_base=["middle/body"])


def test_scene_camera_sees_both_arms():
    # Optical frame (z forward, y down) at x = 1.5 m, 0.5 m up, looking along -x:
    # z_opt = -x_world, x_opt = +y_world, y_opt = -z_world.
    from hirobot.geometry.transforms import matrix_to_quat

    quat = matrix_to_quat(np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float))  # [x y z w]
    cfg = {"defaults": ["robots/ur5e_dual"],
           "sim": {"realtime": False, "ramp_seconds": 0.2},
           "assembly": {"parts": {"left": {"overrides": {"body": MODEL}},
                                  "right": {"overrides": {"body": MODEL}},
                                  "scene": {"device": "camera", "width": 160, "height": 120,
                                            "fovy": 60}},
                        "placement": {"scene": {"pose": [1.5, 0, 0.5, *quat]}}}}
    robot = make_robot(cfg)
    robot.connect()
    try:
        image = robot.get_observation().images.get("scene")
        if image is None:
            pytest.skip("no OpenGL context for MuJoCo rendering on this machine")
        assert image.shape == (120, 160, 3)
        # The arms (left at y = +0.4, right at -0.4) appear right and left of centre
        # in the image (x_opt = +y_world): each half differs from the empty background.
        background = image[0, 0].astype(int)
        diff = np.abs(image.astype(int) - background).sum(axis=2) > 30
        assert diff[:, :70].any() and diff[:, 90:].any()
    finally:
        robot.disconnect()
