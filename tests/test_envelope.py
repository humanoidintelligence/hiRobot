"""A lease's envelope (stricter limits for its own commands) and the arbiter's audit."""

from __future__ import annotations

import logging

import numpy as np
import pytest

from hirobot import (
    Action,
    ControlMode,
    Embodiment,
    PreemptedError,
    Priority,
    RobotBusyError,
    SafetyError,
    make_robot,
)
from hirobot.control.arbiter import CommandArbiter

from .test_layers import FakeWbc, fake_layer  # noqa: F401 (fixture)
from tests.conftest import toy_config


def _cfg(**extra):
    emb = Embodiment(name="toy3", joints=[{"name": f"j{i}"} for i in range(3)], control_hz=50)
    cfg = toy_config(emb, safety={"joint_limit": 1.0, "max_joint_step": 0.5})
    cfg.update(extra)
    return cfg


def _robot(**extra):
    robot = make_robot(_cfg(**extra))
    robot.connect()
    return robot


def _pos(*v):
    return Action(mode=ControlMode.POSITION, joint_target=np.array(v, dtype=float))


def test_envelope_tightens_the_lease_only():
    robot = _robot()
    lease = robot.claim("app", envelope={"joint_limit": 0.3, "max_joint_step": 0.1})
    assert lease.envelope == {"joint_limit": 0.3, "max_joint_step": 0.1}
    lease.send_action(_pos(0.1, 0, 0))
    with pytest.raises(SafetyError, match=r"envelope of lease 'app': .*max_joint_step 0.100"):
        lease.send_action(_pos(0.3, 0, 0))
    with pytest.raises(SafetyError, match=r"envelope of lease 'app': 1 joint target\(s\) exceed"):
        for v in (0.2, 0.3, 0.4):
            lease.send_action(_pos(v, 0, 0))
    lease.release()
    other = robot.claim("other")  # no envelope: the robot's own 0.5 rad step
    other.send_action(_pos(0.7, 0, 0))


def test_looser_items_are_ignored_with_one_warning(caplog):
    robot = _robot()
    with caplog.at_level(logging.WARNING, logger="hirobot.control.safety"):
        lease = robot.claim("app", envelope={"joint_limit": 2.0, "max_joint_step": 0.2,
                                             "max_joint_error": 0.4})
    assert lease.envelope == {"max_joint_step": 0.2, "max_joint_error": 0.4}
    warnings = [r for r in caplog.records if "not stricter" in r.getMessage()]
    assert len(warnings) == 1 and "joint_limit 2" in warnings[0].getMessage()
    lease.release()
    assert robot.claim("loose", envelope={"joint_limit": 5.0}).envelope == {}


def test_allowed_modes_and_gripper_range():
    robot = _robot()
    lease = robot.claim("app", envelope={"allowed_modes": ["position"]})
    assert lease.envelope == {"allowed_modes": ["position"]}
    with pytest.raises(SafetyError, match="mode 'velocity' not allowed"):
        lease.send_action(Action(mode=ControlMode.VELOCITY, joint_target=np.zeros(3)))
    lease.send_action(_pos(0, 0, 0))
    lease.release()
    lease = robot.claim("grip", envelope={"gripper_range": [0.2, 0.8]})
    assert lease.envelope == {"gripper_range": [0.2, 0.8]}


def test_bad_envelopes_fail_before_anything_is_taken():
    robot = _robot()
    for env, match in (({"joint_limt": 0.1}, "unknown key"),
                       ({"max_joint_step": 0}, "must be > 0"),
                       ({"allowed_modes": ["walk"]}, "allowed_modes"),
                       ({"allowed_modes": ["base_velocity"]}, "takes none of them"),
                       ({"gripper_range": [1, 0]}, r"\[low, high\]")):
        with pytest.raises(ValueError, match=match):
            robot.claim("app", envelope=env)
    assert robot._arbiter.holder is None


def test_envelope_does_not_limit_a_layers_own_commands(fake_layer):  # noqa: F811
    layer = {"type": "fake_wbc", "driven": ["j1", "j2"]}
    robot = make_robot(_cfg(layers=[layer], safety={"joint_limit": 1.0}))
    robot.connect()
    robot._inner._joint_pos[1] = 0.6  # the controller holds j1 at 0.6 rad
    lease = robot.claim("app", envelope={"joint_limit": 0.3})
    free = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                  joint_names=["j0"], joint_target=np.array([0.2]))
    lease.send_action(free)  # the PD command below carries j1 = 0.6 > 0.3: not refused
    with pytest.raises(SafetyError, match="envelope"):
        lease.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                                 joint_names=["j0"], joint_target=np.array([0.4])))


def test_audit_records_claims_preemptions_refusals_and_busy_calls():
    robot = _robot()
    robot.send_action(_pos(0.1, 0, 0))  # direct, nobody holding
    app = robot.claim("app", envelope={"max_joint_step": 0.1})
    app.send_action(_pos(0.15, 0, 0))
    with pytest.raises(SafetyError):
        app.send_action(_pos(0.5, 0, 0))
    with pytest.raises(RobotBusyError):
        robot.send_action(_pos(0, 0, 0))
    teleop = robot.claim("teleop", Priority.TAKEOVER)
    with pytest.raises(PreemptedError):
        app.send_action(_pos(0.15, 0, 0))
    teleop.release()
    kinds = [(e.kind, e.lease) for e in robot.command_events()]
    assert kinds == [("claim", "app"), ("refused", "app"), ("busy", None), ("claim", "teleop"),
                     ("preempted", "app"), ("busy", "app"), ("release", "teleop")]
    refused = robot.command_events()[1]
    assert refused.call == "send_action" and "max_joint_step" in refused.detail
    assert refused.priority == "CALLER" and 0 < refused.t < robot.now()  # robot clock
    audit = robot.get_status().safety["audit"]
    assert audit["accepted"] == 2 and audit["refused"] == 1
    assert audit["by_lease"] == {"(direct)": {"accepted": 1, "refused": 0},
                                 "app": {"accepted": 1, "refused": 1}}
    assert audit["holder"] is None
    assert audit["last_refusal"]["lease"] == "app"
    assert app.accepted == 1 and app.refused == 1


def test_audit_ring_is_bounded_and_configurable():
    robot = _robot(arbiter={"audit_size": 3})
    for i in range(5):
        robot.claim(f"l{i}").release()
    events = robot.command_events()
    assert len(events) == 3 and events[-1].lease == "l4"
    with pytest.raises(ValueError, match="audit_size"):
        CommandArbiter(robot, audit_size=0)
    with pytest.raises(ValueError, match="takes require_claim and audit_size"):
        make_robot(_cfg(arbiter={"size": 3}))
