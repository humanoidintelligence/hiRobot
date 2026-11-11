"""Grippers on the ``opening`` interface: embodiment checks, RobotAssembly routing, make_robot.

The driver is the test gripper in ``tests/drivers/end_effector/fake_gripper.py``.
"""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, RobotAssembly, SafetyGate, make_robot
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.devices.end_effector import body_embodiment, build_end_effectors
from hirobot.loading.registry import register_body
from tests.drivers.end_effector.fake_gripper import NAME, FakeGripper, registered

ARM = ["j1", "j2"]


def _emb(**gripper):
    ee = {"name": "gripper", "has_gripper": True, "driver": {"type": NAME, **gripper}}
    return Embodiment(name="arm", kind="arm", control_hz=50,
                      joints=[{"name": n} for n in ARM], end_effectors=[ee])


@pytest.fixture
def gripper_registered():
    with registered():
        yield


class _BodySeesGripper(ToyRobot):
    """A toy body that records the gripper keys it was sent."""

    def __init__(self, embodiment, **_):
        super().__init__(embodiment)
        self.grippers = []

    def _send_action(self, action):
        self.grippers.append(dict(action.gripper))
        super()._send_action(action)


def _assembly():
    emb = _emb()
    body = _BodySeesGripper(body_embodiment(emb))
    ees = build_end_effectors(emb)
    robot = RobotAssembly(body, end_effectors=ees, embodiment=emb)
    robot.connect()
    return robot, body, ees["gripper"]


# --------------------------------------------------------------- embodiment


def test_embodiment_takes_a_driver_without_joints_only_with_has_gripper():
    Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
        {"name": "g", "has_gripper": True, "driver": {"type": "t"}}])
    with pytest.raises(ValueError, match="neither joints"):
        Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
            {"name": "g", "driver": {"type": "t"}}])
    with pytest.raises(ValueError, match="drop has_gripper"):
        Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
            {"name": "g", "joints": ["a"], "has_gripper": True, "driver": {"type": "t"}}])


def test_body_embodiment_hands_the_gripper_to_its_driver(gripper_registered):
    body = body_embodiment(_emb())
    assert body.joint_names == ARM
    (ee,) = body.end_effectors
    assert ee.name == "gripper" and ee.driver is None and not ee.has_gripper


def test_interface_mismatch_is_refused_before_anything_is_built(gripper_registered):
    emb = Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
        {"name": "g", "joints": ["a"], "driver": {"type": NAME}}])
    with pytest.raises(ValueError, match="supports \\['opening'\\], not 'joints'"):
        build_end_effectors(emb)
    hand_emb = Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
        {"name": "g", "has_gripper": True, "driver": {"type": "unitree_handcmd", "side": "left"}}])
    with pytest.raises(ValueError, match="list its joints"):
        build_end_effectors(hand_emb)


# ----------------------------------------------------------------- assembly


def test_observation_reads_the_opening_with_its_stamp_and_extra(gripper_registered):
    robot, _, grip = _assembly()
    grip.opening = 0.25
    obs = robot.get_observation()
    assert obs.gripper == {"gripper": 0.25}
    assert obs.stamps["gripper"].seq >= 1
    assert obs.extra["gripper_object"] == 0.0
    assert obs.joint_pos.shape == (2,)


@pytest.mark.parametrize("mode", [ControlMode.POSITION, ControlMode.VELOCITY, ControlMode.TORQUE])
def test_gripper_goes_out_in_every_joint_mode_and_the_body_never_sees_it(gripper_registered, mode):
    robot, body, grip = _assembly()
    assert robot.action_modes == body.action_modes  # an opening gripper restricts no mode
    robot.send_action(Action(mode=mode, joint_target=np.zeros(2), gripper={"gripper": 0.0}))
    assert grip.sent == [0.0]
    assert body.grippers == [{}]


def test_gripper_alone_reaches_the_gripper(gripper_registered):
    robot, body, grip = _assembly()
    robot.send_action(Action(mode=ControlMode.CARTESIAN, gripper={"gripper": 0.6}))
    assert grip.sent == [0.6]


def test_out_of_range_opening_is_refused_before_anything_moves(gripper_registered):
    robot, body, grip = _assembly()
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.ones(2),
                                 gripper={"gripper": 1.5}))
    assert grip.sent == [] and body.grippers == []


def test_safety_gate_clamps_before_the_gripper_driver(gripper_registered):
    robot, _, grip = _assembly()
    robot._set_safety(SafetyGate(gripper_range=(0.2, 0.8)))
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2),
                             gripper={"gripper": 0.0}))
    assert grip.sent == [pytest.approx(0.2)]


def test_estop_damp_and_disconnect_reach_the_gripper(gripper_registered):
    robot, _, grip = _assembly()
    robot.damp()
    assert grip.damped == 1
    robot.estop()
    assert grip.damped == 2
    robot.disconnect()
    assert grip.damped == 3


def test_hold_leaves_the_gripper_where_it_is(gripper_registered):
    robot, _, grip = _assembly()
    robot.hold()
    assert grip.sent == []


# --------------------------------------------------------------- make_robot


def _tree_cfg(body_driver):
    """The arm of :func:`_emb` as an ``assembly:`` config: a body and a gripper part."""
    return {"assembly": {"name": "arm", "parts": {
        "body": {"device": "body", "control_hz": 50, "floating_base": False,
                 "joints": [{"name": n} for n in ARM], "driver": {"type": body_driver}},
        "gripper": {"device": "end_effector", "interface": "opening",
                    "driver": {"type": NAME}}}}}


def test_make_robot_builds_the_gripper_for_a_real_body(gripper_registered):
    @register_body("test_real_toy")
    class RealToy(_BodySeesGripper):
        simulated = False

    try:
        cfg = _tree_cfg("test_real_toy")
        robot = make_robot(cfg)
        assert isinstance(robot.end_effectors["gripper"], FakeGripper)
        assert robot.body.embodiment.end_effectors[0].has_gripper is False
        robot.connect()
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2),
                                 gripper={"gripper": 0.3}))
        assert robot.end_effectors["gripper"].sent == [0.3]
        assert robot.get_observation().gripper == {"gripper": 0.3}

        ro = make_robot(cfg, overrides={"body": {"driver": {"read_only": True}}})
        assert ro.end_effectors["gripper"].read_only
    finally:
        from hirobot.loading.registry import BODIES

        BODIES._entries.pop("test_real_toy", None)


def test_make_robot_skips_the_gripper_driver_on_a_simulator(gripper_registered):
    robot = make_robot(_tree_cfg("toy"))
    assert not getattr(robot, "end_effectors", {})
