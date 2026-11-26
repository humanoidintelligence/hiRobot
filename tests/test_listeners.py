"""Robot.add_listener: observation and command callbacks on the outermost robot."""

from __future__ import annotations

import logging
import threading

import numpy as np
import pytest

from hirobot import (
    Action,
    CommandRecord,
    ControlMode,
    Embodiment,
    Priority,
    RobotBusyError,
    SafetyError,
    make_robot,
)
from hirobot.drivers.sim.toy import ToyRobot
from tests.conftest import toy_config


def _emb():
    return Embodiment(name="toy", joints=[{"name": f"j{i}", "position_limit": (-1.0, 1.0)}
                                          for i in range(3)], control_hz=50)


def _pos(v=0.1, n=3):
    return Action(mode=ControlMode.POSITION, joint_target=np.full(n, v))


@pytest.fixture
def robot():
    r = make_robot(toy_config(_emb(), safety={"joint_limit": True}))
    r.connect()
    r.reset()
    yield r
    r.disconnect()


def _listen(robot):
    obs, cmds = [], []
    handle = robot.add_listener(on_observation=obs.append, on_command=cmds.append)
    return handle, obs, cmds


def test_observation_listener_gets_each_top_level_read(robot):
    _, obs, _ = _listen(robot)
    got = robot.get_observation()
    assert obs == [got]
    robot.get_observation()
    assert len(obs) == 2


def test_direct_send_is_reported(robot):
    _, _, cmds = _listen(robot)
    action = _pos(0.2)
    robot.send_action(action)
    (rec,) = cmds
    assert isinstance(rec, CommandRecord)
    assert rec.call == "send_action" and rec.action is action
    assert rec.lease is None and rec.priority is None and rec.parts is None
    assert rec.accepted and rec.reason is None
    assert rec.t == pytest.approx(robot.now() - 1 / 50)  # toy time, before the step


def test_lease_send_is_reported_with_name_and_priority(robot):
    _, _, cmds = _listen(robot)
    with robot.claim("teleop", Priority.TAKEOVER) as lease:
        lease.send_action(_pos())
    (rec,) = cmds
    assert rec.lease == "teleop" and rec.priority is Priority.TAKEOVER and rec.accepted


def test_gate_refusal_is_reported(robot):
    _, _, cmds = _listen(robot)
    with pytest.raises(SafetyError):
        robot.send_action(_pos(5.0))
    (rec,) = cmds
    assert not rec.accepted and "exceed" in rec.reason


def test_busy_refusal_is_reported(robot):
    _, _, cmds = _listen(robot)
    with robot.claim("runner"):
        with pytest.raises(RobotBusyError):
            robot.send_action(_pos())
        with pytest.raises(RobotBusyError):
            robot.hold()
    assert [(r.call, r.accepted, r.lease) for r in cmds] == [
        ("send_action", False, None), ("hold", False, None)]
    assert "runner" in cmds[0].reason


def test_hold_is_reported_once_and_its_read_is_not(robot):
    _, obs, cmds = _listen(robot)
    robot.hold()  # reads joint_pos and sends a POSITION action inside
    assert [(r.call, r.action, r.accepted) for r in cmds] == [("hold", None, True)]
    assert obs == []
    with robot.claim("runner") as lease:
        lease.hold()
    assert [r.call for r in cmds] == ["hold", "hold"] and cmds[1].lease == "runner"
    assert obs == []


def test_reset_is_not_reported(robot):
    _, _, cmds = _listen(robot)
    robot.reset()
    assert cmds == []


def test_raising_listener_is_logged_and_harmless(robot, caplog):
    def boom(_):
        raise RuntimeError("listener bug")

    good = []
    bad = robot.add_listener(on_observation=boom, on_command=boom)
    robot.add_listener(on_command=good.append)
    with caplog.at_level(logging.ERROR):
        assert robot.get_observation().joint_pos is not None
        robot.send_action(_pos(0.3))
        robot.send_action(_pos(0.3))
    assert np.allclose(robot.get_observation().joint_pos, 0.3)
    assert len(good) == 2
    assert bad.errors >= 3
    logged = [r for r in caplog.records if "listener" in r.getMessage()]
    assert len(logged) == 1  # rate-limited


def test_remove_stops_callbacks(robot):
    handle, obs, cmds = _listen(robot)
    assert handle.active
    handle.remove()
    handle.remove()  # twice is fine
    assert not handle.active
    robot.get_observation()
    robot.send_action(_pos())
    assert obs == [] and cmds == []


def test_add_listener_needs_a_callable(robot):
    with pytest.raises(TypeError):
        robot.add_listener()
    with pytest.raises(TypeError):
        robot.add_listener(on_command=1)


def test_bare_driver_gets_an_arbiter():
    r = ToyRobot(_emb())
    r.connect()
    obs, cmds = [], []
    r.add_listener(on_observation=obs.append, on_command=cmds.append)
    r.send_action(_pos())
    r.get_observation()
    assert len(obs) == 1 and [c.call for c in cmds] == ["send_action"]


class _Sub(ToyRobot):
    def get_observation(self):
        obs = super().get_observation()  # the wrapped parent: not a second report
        obs.extra["sub"] = True
        return obs


def test_subclass_calling_super_reports_once():
    r = _Sub(_emb())
    r.connect()
    obs = []
    r.add_listener(on_observation=obs.append)
    r.get_observation()
    assert len(obs) == 1 and obs[0].extra["sub"]


def test_part_lease_reports_its_parts():
    arm = {"assembly": {"parts": {"body": {
        "device": "body", "control_hz": 100, "joints": [{"name": f"j{i}"} for i in range(3)],
        "driver": {"type": "toy"}}}}}
    r = make_robot({"assembly": {"parts": {"left": arm, "right": arm}}})
    r.connect()
    r.reset()
    cmds = []
    r.add_listener(on_command=cmds.append)
    with r.claim("left", parts=["left"]) as lease:
        lease.send_action(Action(parts={"left": _pos()}))
    assert cmds[0].parts == ("left",) and cmds[0].accepted


def test_listeners_added_and_removed_while_commanding(robot):
    stop = threading.Event()
    errors = []

    def churn():
        try:
            while not stop.is_set():
                robot.add_listener(on_observation=lambda o: None,
                                   on_command=lambda c: None).remove()
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    t = threading.Thread(target=churn)
    t.start()
    try:
        for _ in range(300):
            robot.send_action(_pos())
            robot.get_observation()
    finally:
        stop.set()
        t.join()
    assert errors == []
