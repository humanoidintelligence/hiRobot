"""Per-joint gate limits and per-gripper ranges (one number, or one value per joint)."""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, SafetyError, SafetyGate, make_robot
from tests.conftest import toy_config


def _emb():
    return Embodiment(name="toy3", joints=[{"name": n} for n in ("a", "b", "c")], control_hz=50)


def _pos(*v, names=None):
    return Action(mode=ControlMode.POSITION, joint_target=np.array(v, dtype=float),
                  joint_names=names)


def _robot(**safety):
    robot = make_robot(toy_config(_emb(), safety=safety))
    robot.connect()
    return robot


def test_joint_limit_per_joint_refuses_and_names_the_joint():
    gate = SafetyGate(num_dofs=3, joint_limit=[1.0, 0.5, 2.0])
    assert gate.check(_pos(0.9, 0.4, 1.9), _emb()).allowed
    verdict = gate.check(_pos(0.9, 0.6, 1.9), _emb())
    assert not verdict.allowed
    assert "'b' at +0.600 rad, limit ±0.500 rad" in verdict.reason


def test_joint_limit_per_joint_clamps_each_joint_to_its_own_bound():
    gate = SafetyGate(num_dofs=3, joint_limit=[1.0, 0.5, 2.0], clamp=True)
    verdict = gate.check(_pos(3.0, -3.0, 3.0), _emb())
    assert verdict.allowed
    assert np.allclose(verdict.action.joint_target, [1.0, -0.5, 2.0])


def test_joint_limit_per_joint_follows_joint_names():
    gate = SafetyGate(num_dofs=3, joint_limit=[1.0, 0.5, 2.0])
    action = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                    joint_target=np.array([1.5, 0.4]), joint_names=["c", "b"])
    assert gate.check(action, _emb()).allowed
    action = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                    joint_target=np.array([0.4, 1.5]), joint_names=["b", "c"])
    assert gate.check(action, _emb()).allowed
    action = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                    joint_target=np.array([1.5]), joint_names=["b"])
    assert "'b'" in gate.check(action, _emb()).reason


def test_a_number_still_works_and_keeps_its_message():
    gate = SafetyGate(num_dofs=3, joint_limit=1.0)
    assert gate.joint_limit == 1.0
    assert "exceed ±1.000 rad" in gate.check(_pos(0, 2.0, 0), _emb()).reason


@pytest.mark.parametrize("kwargs, match", [
    ({"num_dofs": 3, "joint_limit": [1.0, 1.0]}, r"has 2 values for 3 joints"),
    ({"num_dofs": None, "joint_limit": [1.0, 1.0, 1.0]}, r"needs the joint count"),
    ({"num_dofs": 3, "max_joint_step": [0.1, 0.0, 0.1]}, r"must be > 0 for every joint"),
    ({"num_dofs": 3, "max_joint_error": [[0.1]]}, r"a number or a list"),
    ({"num_dofs": 3, "gripper_range": {"left": [1.0, 0.0]}}, r"gripper_range\['left'\]"),
    ({"num_dofs": 3, "gripper_range": [0.0]}, r"must be \[low, high\]"),
])
def test_bad_limits_are_refused_when_the_gate_is_built(kwargs, match):
    with pytest.raises(ValueError, match=match):
        SafetyGate(**kwargs)


def test_max_joint_step_per_joint():
    robot = _robot(max_joint_step=[0.1, 0.5, 0.5])
    robot.send_action(_pos(0.05, 0.4, 0))
    with pytest.raises(SafetyError, match=r"largest: 'a' by \+0.150 rad, limit 0.100 rad"):
        robot.send_action(_pos(0.2, 0.4, 0))
    robot.send_action(_pos(0.1, 0.8, 0))


def test_max_joint_error_per_joint_clamps():
    robot = _robot(max_joint_error=[0.1, 1.0, 1.0], clamp=True)
    robot.send_action(_pos(0.5, 0.5, 0))
    assert np.allclose(robot.get_observation().joint_pos, [0.1, 0.5, 0])


def test_gripper_range_per_end_effector():
    gate = SafetyGate(gripper_range={"left": [0.0, 0.5], "right": [0.0, 1.0]})
    action = Action(mode=ControlMode.POSITION, gripper={"left": np.array(0.9),
                                                        "right": np.array(0.9),
                                                        "other": np.array(5.0)})
    out = gate.check(action, None).action
    assert float(out.gripper["left"]) == 0.5
    assert float(out.gripper["right"]) == 0.9
    assert float(out.gripper["other"]) == 5.0  # not listed: not checked


def test_envelope_per_joint_tightens_only_where_stricter():
    robot = _robot(joint_limit=1.0, max_joint_step=0.5)
    lease = robot.claim("app", envelope={"joint_limit": [0.3, 1.0, 1.0]})
    assert lease.envelope == {"joint_limit": [0.3, 1.0, 1.0]}
    lease.send_action(_pos(0.2, 0.4, 0))
    with pytest.raises(SafetyError, match=r"envelope of lease 'app': .*'a' at \+0.400"):
        lease.send_action(_pos(0.4, 0.4, 0))
    lease.release()
    # Every joint at least as loose as the robot's: ignored.
    assert robot.claim("loose", envelope={"joint_limit": [1.0, 2.0, 1.0]}).envelope == {}


def test_envelope_number_over_per_joint_robot_limit():
    robot = _robot(max_joint_step=[0.1, 0.5, 0.5])
    lease = robot.claim("app", envelope={"max_joint_step": 0.2})
    assert lease.envelope == {"max_joint_step": 0.2}  # stricter on b and c
    with pytest.raises(SafetyError, match="envelope of lease 'app'"):
        lease.send_action(_pos(0, 0.3, 0))


def test_envelope_gripper_range_per_end_effector():
    emb = Embodiment(name="toy3", joints=[{"name": n} for n in ("a", "b", "c")], control_hz=50,
                     end_effectors=[{"name": "left", "has_gripper": True},
                                    {"name": "right", "has_gripper": True}])
    robot = make_robot(toy_config(emb, safety={"gripper_range": {"left": [0.0, 1.0]}}))
    robot.connect()
    assert robot.safety.gripper_range == {"left": (0.0, 1.0)}
    lease = robot.claim("app", envelope={"gripper_range": {"left": [0.0, 0.5], "right": [0, 0.2]}})
    assert lease.envelope == {"gripper_range": {"left": [0.0, 0.5], "right": [0.0, 0.2]}}
    lease.release()
    assert robot.claim("same", envelope={"gripper_range": {"left": [0.0, 1.0]}}).envelope == {}
