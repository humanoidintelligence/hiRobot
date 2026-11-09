"""The safety gate validates, clamps, and refuses — the robot's last check."""

from __future__ import annotations

import types

import numpy as np

from hirobot import SafetyGate


class _Action:
    """Minimal stand-in for hiRobot's Action (keeps the test focused on the gate)."""

    def __init__(self, joint_target=None, gripper=None):
        self.joint_target = None if joint_target is None else np.asarray(joint_target, dtype=float)
        self.gripper = gripper or {}


def _embodiment(n):
    return types.SimpleNamespace(num_dofs=n)


def test_wrong_dof_count_is_refused():
    gate = SafetyGate.from_embodiment(_embodiment(7))
    verdict = gate.check(_Action(joint_target=[0.0] * 5))
    assert not verdict.allowed
    assert "expects 7" in verdict.reason


def test_out_of_range_is_clamped_when_enabled():
    gate = SafetyGate(num_dofs=3, joint_limit=1.0, clamp=True)
    verdict = gate.check(_Action(joint_target=[2.0, -2.0, 0.5]))
    assert verdict.allowed
    assert np.allclose(verdict.action.joint_target, [1.0, -1.0, 0.5])


def test_out_of_range_is_refused_when_clamp_off():
    gate = SafetyGate(num_dofs=3, joint_limit=1.0, clamp=False)
    verdict = gate.check(_Action(joint_target=[2.0, 0.0, 0.0]))
    assert not verdict.allowed
    assert "exceed" in verdict.reason


def test_non_finite_is_refused():
    gate = SafetyGate(num_dofs=2)
    verdict = gate.check(_Action(joint_target=[0.0, np.inf]))
    assert not verdict.allowed


def test_gripper_is_clamped_into_range():
    gate = SafetyGate(num_dofs=None, gripper_range=(0.0, 1.0))
    verdict = gate.check(_Action(gripper={"left": np.asarray(1.5)}))
    assert verdict.allowed
    assert float(verdict.action.gripper["left"]) == 1.0


def test_defaults_skip_bounds_but_keep_structural_checks():
    # A gate with no limits configured leaves in-range values alone and does not
    # impose joint or gripper bounds (robots differ: some grippers use -1, some
    # joints exceed ±pi), but still enforces DoF count and finiteness.
    gate = SafetyGate.from_embodiment(_embodiment(3))
    ok = gate.check(_Action(joint_target=[5.0, -5.0, 0.0], gripper={"g": np.asarray(-1.0)}))
    assert ok.allowed
    assert np.allclose(ok.action.joint_target, [5.0, -5.0, 0.0])
    assert float(ok.action.gripper["g"]) == -1.0
    assert not gate.check(_Action(joint_target=[0.0, 0.0])).allowed  # wrong DoF count


def test_send_action_enforces_gate_and_raises():
    """The gate is enforced inside send_action, not by the caller."""
    import pytest

    from hirobot import SafetyError
    from hirobot.drivers.sim.toy import ToyRobot
    from hirobot.embodiment import Embodiment

    emb = Embodiment(
        name="fake", kind="arm", control_hz=20, floating_base=False,
        joints=[{"name": f"j{i}"} for i in range(3)],
    )
    robot = ToyRobot(emb)
    robot.set_safety(SafetyGate.from_embodiment(emb, joint_limit=1.0))  # clamp off by default
    robot.connect()

    from hirobot import Action

    with pytest.raises(SafetyError, match="exceed"):
        robot.send_action(Action(joint_target=np.array([2.0, 0.0, 0.0])))

    robot.send_action(Action(joint_target=np.array([0.5, -0.5, 0.0])))  # in range: passes
    assert np.allclose(robot.get_observation().joint_pos, [0.5, -0.5, 0.0])


def test_make_robot_attaches_a_gate():
    from hirobot import make_robot
    from hirobot.config import available_configs, load_config

    sim_cfgs = [c for c in available_configs("robots") if load_config(c)["body"]["driver"] == "toy"]
    assert sim_cfgs, "expected at least one sim config"
    robot = make_robot(sim_cfgs[0])
    assert robot.safety is not None
    assert robot.safety.num_dofs == robot.embodiment.num_dofs
