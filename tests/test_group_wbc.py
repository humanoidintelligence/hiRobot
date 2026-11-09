"""RobotGroup with a whole-body-control member from hiWBC (the ``wbc`` layer, ``echo`` profile).

Needs hiWBC installed (it registers the ``wbc`` layer through the ``hirobot.drivers``
entry point); skipped otherwise. The echo controller holds a default pose and needs no
checkpoint. The G1 is the toy sim, except in the last test (MuJoCo, needs the MJCF).
"""

from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("hiwbc")
from hiwbc.configs.echo.default import G1_29

from hirobot import Action, ControlMode, RobotFaultError, RobotGroup, make_robot

LOWER = list(G1_29[:15])   # legs and waist: driven by the controller
ARM_JOINTS = [f"j{i}" for i in range(3)]


def _g1(**layer):
    return make_robot({
        "embodiment": {"name": "g1", "control_hz": 50, "joints": [{"name": n} for n in G1_29]},
        "body": {"driver": "toy"},
        "layers": [{"type": "wbc", "profile": "echo/default", "start_seconds": 0.0,
                    "joint_names": LOWER, **layer}]})


def _arm():
    return make_robot({
        "embodiment": {"name": "arm", "kind": "arm", "floating_base": False, "control_hz": 50,
                       "joints": [{"name": n} for n in ARM_JOINTS]},
        "body": {"driver": "toy"}})


def _group():
    group = RobotGroup({"g1": _g1(), "arm": _arm()})
    group.connect()
    return group


def _ticks(group):
    return group.get_status().extra["g1/wbc"]["ticks"]


def _q(group, name):
    return group.get_observation().joint_pos[list(group.embodiment.joint_names).index(name)]


def test_wbc_member_capabilities_reach_the_group():
    group = _group()
    try:
        g1 = group.member("g1")
        assert group.embodiment.floating_base  # the one floating member
        # The controller leaves the upper body free; a plain arm takes no non-joint
        # mode, so it has no direct joints.
        assert set(group.direct_joints) == {f"g1/{n}" for n in G1_29[15:]}
        assert group.action_modes == ()  # no mode both members take: use Action.members
        assert g1.simulated
        assert group.get_status().extra["g1/wbc"]["profile"] == "echo/default"
    finally:
        group.disconnect()


def test_controller_ticks_when_only_the_arm_is_commanded():
    group = _group()
    try:
        before = _ticks(group)
        for _ in range(5):
            group.send_action(Action(members={
                "arm": Action(mode=ControlMode.POSITION, joint_target=np.ones(3))}))
        # idle_step: one controller tick per group send, though the G1 was not named.
        assert _ticks(group) == before + 5
        np.testing.assert_allclose(group.get_observation().joint_pos[-3:], 1.0)
    finally:
        group.disconnect()


def test_flat_base_velocity_goes_to_the_wbc_member():
    group = _group()
    try:
        before = _ticks(group)
        group.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0])))
        status = group.get_status().extra["g1/wbc"]
        assert status["ticks"] == before + 1 and status["reference"] == "velocity"
    finally:
        group.disconnect()


def test_members_action_mixes_wbc_and_joint_modes():
    group = _group()
    try:
        group.send_action(Action(members={
            # Walk command for the controller, plus a direct target for an upper-body joint.
            "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                         joint_names=["right_elbow"], joint_target=np.array([0.5])),
            "arm": Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2, 0.3])),
        }))
        assert _q(group, "g1/right_elbow") == pytest.approx(0.5)
        np.testing.assert_allclose(group.get_observation().joint_pos[-3:], [0.1, 0.2, 0.3])
    finally:
        group.disconnect()


def test_wbc_member_refuses_joint_modes_and_nothing_is_sent():
    group = _group()
    try:
        before = _ticks(group)
        with pytest.raises(Exception, match="does not take mode 'position'"):
            group.send_action(Action(members={
                "g1": Action(mode=ControlMode.POSITION, joint_target=np.zeros(29)),
                "arm": Action(mode=ControlMode.POSITION, joint_target=np.ones(3))}))
        assert _ticks(group) == before  # refused before anything was sent
        np.testing.assert_allclose(group.get_observation().joint_pos[-3:], 0.0)
        # A controller-owned joint named beside the walk command is refused too.
        with pytest.raises(Exception, match="not free to command directly"):
            group.send_action(Action(members={
                "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                             joint_names=["left_knee"], joint_target=np.array([0.3]))}))
    finally:
        group.disconnect()


def test_wbc_member_is_owned_by_the_group():
    g1, arm = _g1(), _arm()
    group = RobotGroup({"g1": g1, "arm": arm})
    group.connect()
    try:
        with pytest.raises(RuntimeError, match="goes through the group"):
            g1.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))
    finally:
        group.disconnect()


def test_controller_failure_holds_the_group():
    g1, arm = _g1(), _arm()
    group = RobotGroup({"g1": g1, "arm": arm})
    group.connect()
    try:
        def boom(*_a, **_k):
            raise RuntimeError("controller blew up")

        g1.controller.step = boom
        with pytest.raises(RobotFaultError):
            group.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))
        assert group.fault is not None
        with pytest.raises(RobotFaultError):
            group.send_action(Action(members={
                "arm": Action(mode=ControlMode.POSITION, joint_target=np.ones(3))}))
        np.testing.assert_allclose(group.get_observation().joint_pos[-3:], 0.0)
    finally:
        group.disconnect()


G1_MJCF = os.path.expanduser("~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")


@pytest.mark.skipif(not os.path.isfile(G1_MJCF), reason="unitree_ros G1 MJCF not present")
def test_mujoco_g1_with_echo_wbc_holds_its_pose_beside_an_arm():
    pytest.importorskip("mujoco")
    default = np.zeros(29)
    default[[3, 9]] = 0.3   # knees bent
    g1 = make_robot("unitree_g1_mujoco", body_overrides={"realtime": False, "ramp_seconds": 0.0})
    from hiwbc.robot import WbcRobot

    g1 = WbcRobot(g1, "echo/default", default_pos=list(default), start_seconds=0.5)
    group = RobotGroup({"g1": g1, "arm": _arm()})
    group.connect()
    try:
        sim = g1.inner
        t0 = sim.data.time
        for _ in range(50):  # 1 s at 50 Hz, the arm only
            group.send_action(Action(members={
                "arm": Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.2))}))
        assert sim.data.time - t0 == pytest.approx(50 * sim.control_period, rel=1e-6)
        q = group.get_observation().joint_pos
        names = list(group.embodiment.joint_names)
        knees = [names.index("g1/left_knee"), names.index("g1/right_knee")]
        np.testing.assert_allclose(q[knees], 0.3, atol=0.05)
    finally:
        group.disconnect()
