"""Command arbiter: one holder at a time, preemption by priority."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from hirobot import (
    Action,
    ControlMode,
    Embodiment,
    PreemptedError,
    Priority,
    RobotBusyError,
    make_robot,
)
from hirobot.control.arbiter import CommandArbiter
from hirobot.drivers.sim.toy import ToyRobot
from tests.conftest import toy_config


def _pos(n=3, v=0.1):
    return Action(mode=ControlMode.POSITION, joint_target=np.full(n, v))


class CountingToy(ToyRobot):
    def __init__(self):
        super().__init__(Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)],
                                    control_hz=50))
        self.holds = 0

    def hold(self):
        self.holds += 1
        super().hold()


@pytest.fixture
def robot():
    r = CountingToy()
    r.connect()
    return r


def test_nobody_holding_lets_everyone_command(robot):
    robot.send_action(_pos())
    robot.hold()
    robot.reset()


def test_held_robot_refuses_direct_calls_and_the_lease_commands(robot):
    with robot.claim("runner") as r:
        for call in (lambda: robot.send_action(_pos()), robot.hold, robot.reset,
                     lambda: robot.run_robot_skill("x")):
            with pytest.raises(RobotBusyError, match="runner"):
                call()
        r.send_action(_pos(v=0.3))
        r.hold()  # nested send_action inside hold() passes
        r.reset()
        assert np.allclose(robot.get_observation().joint_pos, 0.0)
        robot.get_observation()
        robot.get_status()
        robot.now()
    robot.send_action(_pos())  # released: open again


def test_estop_is_never_arbitrated(robot):
    with robot.claim("runner"):
        robot.estop()
    assert robot.fault is not None


def test_same_or_lower_priority_is_refused(robot):
    with robot.claim("a", Priority.LAYER_SKILL):
        with pytest.raises(RobotBusyError, match="'a'"):
            robot.claim("b", Priority.LAYER_SKILL)
        with pytest.raises(RobotBusyError):
            robot.claim("c", Priority.CALLER)


def test_higher_priority_preempts_holds_and_calls_on_preempt(robot):
    seen = []
    runner = robot.claim("runner", on_preempt=lambda lease: seen.append(lease.name))
    runner.send_action(_pos())
    holds = robot.holds
    teleop = robot.claim("teleop", Priority.TAKEOVER)
    assert seen == ["runner"] and robot.holds == holds + 1
    assert runner.preempted and runner.preempted_by == "teleop" and not runner.active
    with pytest.raises(PreemptedError, match="teleop"):
        runner.send_action(_pos())
    teleop.send_action(_pos(v=0.2))
    teleop.release()
    # Not given back: the preempted lease stays dead; the runner claims again.
    with pytest.raises(PreemptedError):
        runner.send_action(_pos())
    runner.release()  # harmless after a preemption
    with robot.claim("runner") as again:
        again.send_action(_pos())


def test_preemption_still_succeeds_when_hold_fails(robot):
    runner = robot.claim("runner")
    robot.estop()  # fault: hold() raises
    teleop = robot.claim("teleop", Priority.TAKEOVER)
    assert teleop.active and runner.preempted


def test_a_failing_on_preempt_does_not_block_the_takeover(robot):
    def boom(_lease):
        raise RuntimeError("boom")

    robot.claim("runner", on_preempt=boom)
    assert robot.claim("teleop", Priority.TAKEOVER).active


def test_released_lease_refuses(robot):
    lease = robot.claim("runner")
    lease.release()
    with pytest.raises(RobotBusyError, match="released"):
        lease.send_action(_pos())


def test_require_claim(robot):
    robot._arbiter = CommandArbiter(robot, require_claim=True)
    with pytest.raises(RobotBusyError, match="claim first"):
        robot.send_action(_pos())
    with robot.claim("runner") as r:
        r.send_action(_pos())


def test_lease_works_from_another_thread(robot):
    lease = robot.claim("runner")
    box = {}

    def run():
        try:
            lease.send_action(_pos(v=0.4))
        except Exception as exc:  # noqa: BLE001
            box["exc"] = exc

    t = threading.Thread(target=run)
    t.start()
    t.join()
    assert "exc" not in box
    assert np.allclose(robot.get_observation().joint_pos, 0.4)


def _toy_cfg(**extra):
    emb = Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)], control_hz=50)
    return toy_config(emb, **extra)


def test_make_robot_attaches_one_arbiter_on_the_outermost_robot():
    robot = make_robot(_toy_cfg())
    assert isinstance(robot._arbiter, CommandArbiter) and robot._arbiter.robot is robot
    assert not robot._arbiter.require_claim
    strict = make_robot(_toy_cfg(arbiter={"require_claim": True}))
    assert strict._arbiter.require_claim
    with pytest.raises(ValueError, match="arbiter"):
        make_robot(_toy_cfg(arbiter={"bogus": 1}))


def test_two_leases_take_turns(robot):
    for i in range(3):
        with robot.claim(f"caller{i % 2}") as r:
            r.send_action(_pos(v=0.1 * i))
            assert np.allclose(robot.get_observation().joint_pos, 0.1 * i)


def test_runner_is_refused_during_a_takeover_on_its_own_thread(robot):
    runner = robot.claim("runner")
    started, takeover_done, box = threading.Event(), threading.Event(), {}

    def loop():
        started.set()
        takeover_done.wait(1.0)
        try:
            runner.send_action(_pos(v=0.9))
        except Exception as exc:  # noqa: BLE001
            box["exc"] = exc

    t = threading.Thread(target=loop)
    t.start()
    started.wait(1.0)
    with robot.claim("teleop", Priority.TAKEOVER) as teleop:
        takeover_done.set()
        t.join()
        teleop.send_action(_pos(v=0.2))
    assert isinstance(box.get("exc"), PreemptedError)
    assert np.allclose(robot.get_observation().joint_pos, 0.2)


# ------------------------------------------------------------ leases on parts

_ARM = {"device": "body", "control_hz": 100, "joints": [{"name": f"j{i}"} for i in range(3)],
        "driver": {"type": "toy"}}
_BASE = {"device": "body", "control_hz": 50, "floating_base": True, "joints": [],
         "driver": {"type": "toy", "mobile_base": True}}


def _two_arms(**safety):
    arm = {"assembly": {"parts": {"body": _ARM}}}
    cfg = {"assembly": {"parts": {"left": arm, "right": arm,
                                  "cell": {"assembly": {"parts": {"a": arm, "b": arm}}}}}}
    if safety:
        cfg["safety"] = safety
    r = make_robot(cfg)
    r.connect()
    r.reset()
    return r


def _part(name, v):
    return Action(parts={name: _pos(v=v)})


def test_part_leases_on_different_parts_are_held_together():
    robot = _two_arms()
    left, right = robot.claim("left", parts=["left"]), robot.claim("right", parts=["right"])
    left.send_action(_part("left", 0.2))
    right.send_action(_part("right", 0.3))
    q = robot.get_observation().joint_pos
    assert np.allclose(q[:3], 0.2) and np.allclose(q[3:6], 0.3)
    assert robot.get_status().safety["audit"]["holders"] == {"left": "left", "right": "right"}
    with pytest.raises(RobotBusyError, match="also commands"):
        left.send_action(_part("right", 0.0))  # outside the lease
    with pytest.raises(RobotBusyError, match="also commands"):
        left.send_action(_pos(n=12, v=0.0))  # a flat action covers every part
    with pytest.raises(RobotBusyError, match="goes through its lease"):
        robot.send_action(_part("left", 0.0))
    robot.send_action(Action(parts={"cell": _pos(n=6, v=0.1)}))  # nobody holds the cell: a direct call passes
    with pytest.raises(RobotBusyError, match="cannot take it"):
        robot.claim("other", parts=["left"])
    with pytest.raises(RobotBusyError, match="cannot take it"):
        robot.claim("whole")  # overlaps both


def test_subtree_paths_overlap_by_prefix():
    robot = _two_arms()
    cell = robot.claim("cell", parts=["cell"])
    with pytest.raises(RobotBusyError):
        robot.claim("a", parts=["cell/a"])
    cell.send_action(Action(parts={"cell": Action(parts={"a": _pos(v=0.1), "b": _pos(v=0.2)})}))
    cell.release()
    a = robot.claim("a", parts=["cell/a"])
    b = robot.claim("b", parts=["cell/b"])
    with pytest.raises(RobotBusyError):
        a.send_action(Action(parts={"cell": Action(parts={"b": _pos(v=0.0)})}))
    a.release(), b.release()
    with pytest.raises(ValueError, match="no part 'nope'"):
        robot.claim("x", parts=["nope"])


def test_preemption_holds_only_the_preempted_parts():
    robot = _two_arms()
    left, right = robot.claim("left", parts=["left"]), robot.claim("right", parts=["right"])
    held = []
    left.on_preempt = lambda lease: held.append(lease.name)
    left.send_action(_part("left", 0.2))
    teleop = robot.claim("teleop", Priority.TAKEOVER, parts=["left"])
    assert left.preempted and held == ["left"] and right.active
    with pytest.raises(PreemptedError):
        left.send_action(_part("left", 0.1))
    right.send_action(_part("right", 0.3))
    events = [(e.kind, e.lease, e.parts) for e in robot.command_events()]
    assert ("preempted", "left", ["left"]) in events
    teleop.release()
    with pytest.raises(PreemptedError):  # not given back
        left.send_action(_part("left", 0.1))


def test_part_hold_and_reset_stay_in_the_part():
    robot = _two_arms(max_joint_step=0.5)
    left, right = robot.claim("left", parts=["left"]), robot.claim("right", parts=["right"])
    left.send_action(_part("left", 0.2))
    right.send_action(_part("right", 0.3))
    left.reset()
    q = robot.get_observation().joint_pos
    assert np.allclose(q[:3], 0.0) and np.allclose(q[3:6], 0.3)
    right.hold()
    assert np.allclose(robot.get_observation().joint_pos[3:6], 0.3)
    # The right arm keeps its step reference: a jump from 0.3 is still refused.
    from hirobot.control.safety import SafetyError

    with pytest.raises(SafetyError, match="max_joint_step"):
        right.send_action(_part("right", 0.9))


def test_envelope_limits_only_the_lease_joints():
    robot = _two_arms(max_joint_step=0.5)
    left = robot.claim("left", parts=["left"], envelope={"max_joint_step": 0.05})
    right = robot.claim("right", parts=["right"])
    from hirobot.control.safety import SafetyError

    with pytest.raises(SafetyError, match="envelope of lease 'left'"):
        left.send_action(_part("left", 0.2))
    right.send_action(_part("right", 0.2))


def test_base_lease_and_arm_lease_on_a_mobile_robot():
    robot = make_robot({"assembly": {"parts": {
        "base": _BASE, "arm": {"assembly": {"parts": {"body": _ARM}}}}}})
    robot.connect()
    robot.reset()
    nav = robot.claim("nav", parts=["base"])
    arm = robot.claim("arm", parts=["arm"])
    nav.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.5, 0, 0])))
    arm.send_action(Action(parts={"arm": _pos(v=0.1)}))
    with pytest.raises(RobotBusyError):
        arm.send_action(Action(mode=ControlMode.BASE_VELOCITY,
                               base_velocity=np.array([0.1, 0, 0])))
    assert robot.get_observation().base_position[0] > 0


def test_robot_without_parts_has_one_part(robot):
    with pytest.raises(ValueError, match="no parts"):
        robot.claim("x", parts=["left"])
    lease = robot.claim("x", parts=[""])  # "" is the whole robot
    assert lease.parts is None
