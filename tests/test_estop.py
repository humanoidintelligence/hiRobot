"""``Robot.estop()``: preempts long calls from another thread, orders against sends,
and reaches every part of a composite robot."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from hirobot import (
    Action,
    ControlMode,
    RobotFaultError,
    RobotStoppedError,
    get_body,
    load_embodiment,
    make_robot,
)
from hirobot.assembly import RobotAssembly
from hirobot.core.robot import Robot
from tests.drivers.body.unitree.sdk2 import fake_sdk as unitree_fake
from tests.drivers.body.ur.rtde import fake_sdk as ur_fake


def _in_thread(fn):
    """Run ``fn`` on a thread; return (thread, box) with its exception in ``box``."""
    box = {}

    def run():
        try:
            box["result"] = fn()
        except BaseException as exc:  # noqa: BLE001 - handed back to the test
            box["error"] = exc

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t, box


def _unitree(monkeypatch, ramp_seconds=5.0):
    bus = unitree_fake.install(monkeypatch, unitree_fake.low_state([0.1] * 35))
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"), ramp_seconds=ramp_seconds,
                                     damping_kd=8.0)
    robot.connect()
    return robot, bus


def _damped(cmd):
    return all(kp == 0.0 and kd == 8.0 for _, _, _, _, _, kp, kd in cmd[:29])


def test_estop_cuts_a_unitree_reset_ramp_short(monkeypatch):
    robot, bus = _unitree(monkeypatch)  # a 5 s ramp
    t, box = _in_thread(robot.reset)
    while len(bus.commands()) < 3:
        time.sleep(0.001)
    t0 = time.monotonic()
    robot.estop()
    t.join(timeout=1.0)
    assert not t.is_alive() and time.monotonic() - t0 < 0.5
    assert isinstance(box.get("error"), RobotStoppedError)
    time.sleep(0.05)  # no ramp step may follow the damping command
    assert _damped(bus.commands()[-1])
    assert robot.fault is not None  # the interrupted reset does not clear it
    with pytest.raises(RobotFaultError):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(29)))
    robot._ramp_seconds = 0.0
    robot.reset()
    assert robot.fault is None


def test_estop_cuts_a_unitree_connect_wait_short(monkeypatch):
    unitree_fake.install(monkeypatch, None)  # no LowState ever arrives
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"))
    t, box = _in_thread(robot.connect)
    time.sleep(0.05)
    robot.estop()
    t.join(timeout=1.0)
    assert not t.is_alive()
    assert isinstance(box.get("error"), RobotStoppedError)


def test_estop_with_builtin_skills_stops_the_skill(monkeypatch):
    bus = unitree_fake.install(monkeypatch, unitree_fake.low_state([0.0] * 35))
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"), builtin_skills="g1")
    robot.connect()
    stopped = []
    monkeypatch.setattr(robot, "stop_onboard_skill", lambda: stopped.append(True))
    robot.estop()
    assert stopped == [True] and bus.commands() == []


def test_no_command_goes_out_after_estop():
    """A send under way finishes before the stop; none starts after it."""
    log = []
    entered = threading.Event()
    release = threading.Event()

    class Slow(type(make_robot("unitree_g1"))):
        def _send_action(self, action):
            entered.set()
            release.wait(1.0)
            log.append("send")

        def damp(self):
            log.append("damp")

    robot = Slow(load_embodiment("unitree_g1"))
    robot.connect()
    robot.reset()
    action = Action(mode=ControlMode.POSITION, joint_target=np.zeros(29))
    t, _ = _in_thread(lambda: robot.send_action(action))
    entered.wait(1.0)
    s, _ = _in_thread(robot.estop)
    time.sleep(0.05)
    assert log == []  # the stop waits for the send in flight
    release.set()
    t.join(1.0)
    s.join(1.0)
    assert log == ["send", "damp"]
    with pytest.raises(RobotFaultError):
        robot.send_action(action)
    assert log == ["send", "damp"]


def test_estop_does_not_wait_forever_for_a_stuck_send(monkeypatch):
    import hirobot.core.robot as robot_mod

    monkeypatch.setattr(robot_mod, "ESTOP_LOCK_TIMEOUT_S", 0.05)
    stuck = threading.Event()
    damped = []

    class Stuck(type(make_robot("unitree_g1"))):
        def _send_action(self, action):
            stuck.set()
            time.sleep(1.0)

        def damp(self):
            damped.append(True)

    robot = Stuck(load_embodiment("unitree_g1"))
    robot.connect()
    _in_thread(lambda: robot.send_action(
        Action(mode=ControlMode.POSITION, joint_target=np.zeros(29))))
    stuck.wait(1.0)
    t0 = time.monotonic()
    robot.estop()
    assert damped == [True] and time.monotonic() - t0 < 0.5


def test_driver_without_damp_still_refuses_commands():
    robot = make_robot("unitree_g1")  # toy: no damp()
    robot.connect()
    robot.reset()
    robot.estop()
    with pytest.raises(RobotFaultError, match="estop"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(29)))
    robot.reset()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(29)))


def test_ur_estop_stops_the_move_home(monkeypatch):
    arm = ur_fake.install(monkeypatch)
    arm.progress = [0] * 10_000  # a moveJ that would run for a long time
    robot = get_body("ur_rtde")(load_embodiment("ur_ur5e"), home_q=[0.0] * 6)
    robot.connect()
    t, box = _in_thread(robot.reset)
    time.sleep(0.05)
    robot.estop()
    t.join(1.0)
    assert not t.is_alive() and isinstance(box.get("error"), RobotStoppedError)
    names = [c[0] for c in arm.calls]
    assert names[-3:] == ["servoStop", "speedStop", "stopJ"]


def test_estop_reaches_the_hands_of_an_assembly(monkeypatch):
    from tests.test_end_effectors import _assembly

    robot, body, bus = _assembly(monkeypatch)
    robot.connect()
    robot.estop()
    assert body.damped == 1
    for hand in robot.end_effectors.values():  # Dex3 damp: kp 0 on every finger
        assert all(m[2] == 0.0 for m in bus.sent[hand.cmd_topic][-1])
    assert robot.fault is not None
    for hand in robot.end_effectors.values():
        hand._ramp_seconds = 0.0
    robot.reset()
    assert robot.fault is None


def test_estop_cuts_a_dex3_reset_ramp_short(monkeypatch):
    from tests.test_end_effectors import _assembly

    robot, _, bus = _assembly(monkeypatch)
    robot.connect()
    hand = robot.end_effectors["right_hand"]
    hand._ramp_seconds, hand._ramp_period = 5.0, 0.01
    t, box = _in_thread(hand.reset)
    while len(bus.sent[hand.cmd_topic]) < 3:
        time.sleep(0.001)
    robot.estop()
    t.join(1.0)
    assert not t.is_alive() and isinstance(box.get("error"), RobotStoppedError)
    time.sleep(0.03)
    assert all(m[2] == 0.0 for m in bus.sent[hand.cmd_topic][-1])


def test_group_estop_stops_every_member_even_if_one_fails():
    left, right = make_robot("ur_ur5e"), make_robot("ur_ur5e")

    def boom():
        raise RuntimeError("bus down")

    left.damp = boom  # type: ignore[attr-defined]
    stopped = []
    right.damp = lambda: stopped.append(True)  # type: ignore[attr-defined]
    group = RobotAssembly(None, parts={"left": left, "right": right})
    group.connect()
    group.reset()
    with pytest.raises(RuntimeError, match="bus down"):
        group.estop()
    assert stopped == [True]
    assert group.fault is not None and left.fault is not None and right.fault is not None
    with pytest.raises(RobotFaultError):
        group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(12)))
    group.reset()
    assert group.fault is None and left.fault is None and right.fault is None


def test_estop_on_a_group_member_needs_no_group_call():
    """An e-stop skips the group's owner check: anyone may stop a member."""
    left, right = make_robot("ur_ur5e"), make_robot("ur_ur5e")
    group = RobotAssembly(None, parts={"left": left, "right": right})
    group.connect()
    left.estop()
    assert left.fault is not None


def test_subclass_reset_clears_the_fault_without_calling_clear_fault():
    class Plain(type(make_robot("ur_ur5e"))):
        def reset(self, seed=None):
            pass  # no _clear_fault() here

    robot = Plain(load_embodiment("ur_ur5e"))
    robot.estop()
    robot.reset()
    assert robot.fault is None
    assert Plain.reset.__wrapped__.__qualname__.endswith("Plain.reset")
    assert isinstance(robot, Robot)
