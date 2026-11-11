"""Command watchdog: a velocity command the caller stops refreshing is stopped."""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, get_body, load_embodiment
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.control.watchdog import CommandWatchdog
from tests.drivers.body.unitree.sdk2 import fake_sdk as unitree_fake
from tests.drivers.body.ur.rtde import fake_sdk as ur_fake


def _vel(n, v=0.5):
    return Action(mode=ControlMode.VELOCITY, joint_target=np.full(n, v))


def _emb(n=3):
    return Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(n)], control_hz=50)


# -------------------------------------------------------------- the watchdog


def test_watchdog_fires_once_per_stale_command_and_skips_other_modes():
    t = [0.0]
    stopped = []
    dog = CommandWatchdog("x", 0.2, stopped.append, clock=lambda: t[0])
    dog.feed(ControlMode.VELOCITY)
    t[0] = 0.1
    assert not dog.check()
    t[0] = 0.3
    assert dog.check() and stopped == [ControlMode.VELOCITY]
    t[0] = 1.0
    assert not dog.check()  # once per stale command
    dog.feed(ControlMode.BASE_VELOCITY)
    dog.feed(ControlMode.POSITION)  # replaces the velocity: nothing to stop
    t[0] = 2.0
    assert not dog.check() and dog.timeouts == 1


def test_watchdog_off():
    for timeout in (None, 0):
        dog = CommandWatchdog("x", timeout, lambda m: pytest.fail("fired"), clock=lambda: 9.0)
        dog.feed(ControlMode.VELOCITY)
        assert not dog.enabled and not dog.check()


# ------------------------------------------------------------- simulators


def test_toy_stops_a_stale_velocity_and_keeps_a_fresh_one():
    robot = ToyRobot(_emb())
    robot.connect()
    robot.send_action(_vel(3))
    for _ in range(5):  # 0.1 s: still moving
        robot._idle_step()
    moving = robot.get_observation().joint_pos.copy()
    assert np.all(moving > 0) and np.allclose(robot.get_observation().joint_vel, 0.5)
    for _ in range(10):  # past 0.2 s
        robot._idle_step()
    assert np.allclose(robot.get_observation().joint_vel, 0.0)
    still = robot.get_observation().joint_pos.copy()
    robot._idle_step()
    assert np.allclose(robot.get_observation().joint_pos, still)
    # A fresh command runs as usual; the robot is not in a fault state.
    assert robot.fault is None
    robot.send_action(_vel(3, 1.0))
    assert np.allclose(robot.get_observation().joint_vel, 1.0)


def test_toy_watchdog_can_be_turned_off():
    robot = ToyRobot(_emb(), command_timeout_s=None)
    robot.connect()
    robot.send_action(_vel(3))
    for _ in range(50):
        robot._idle_step()
    assert robot.command_timeout_s is None
    assert np.allclose(robot.get_observation().joint_vel, 0.5)


ARM_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="link0" pos="0 0 0.1">
      <joint name="j0" axis="0 0 1" damping="0"/>
      <geom size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator><motor joint="j0" ctrlrange="-100 100"/></actuator>
</mujoco>
"""


def test_mujoco_stops_a_stale_velocity(tmp_path):
    pytest.importorskip("mujoco")
    path = tmp_path / "arm.xml"
    path.write_text(ARM_XML)
    emb = Embodiment(name="arm", kind="arm", floating_base=False, control_hz=50,
                     joints=[{"name": "j0"}])
    robot = get_body("mujoco")(emb, model_path=str(path), realtime=False, viewer=False,
                               default_kd=0.02)  # I = 0.001 kg m^2: tau = 0.05 s, stable at 2 ms
    robot.connect()
    for _ in range(10):
        robot.send_action(_vel(1, 1.0))
    assert robot.get_observation().joint_vel[0] == pytest.approx(1.0, abs=0.1)
    for _ in range(50):  # 1 s with no new command; the watchdog drops it after 0.2 s
        robot._idle_step()
    assert abs(robot.get_observation().joint_vel[0]) < 0.05
    assert robot._watchdog.timeouts == 1 and robot.fault is None
    robot.send_action(_vel(1, 1.0))  # a new command runs as usual
    robot.disconnect()


# ---------------------------------------------------------------- hardware


def _wait_for(pred, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def test_ur_speed_stop_after_a_stale_speedj(monkeypatch):
    arm = ur_fake.install(monkeypatch)
    robot = get_body("ur_rtde")(load_embodiment("ur_ur5e"), command_timeout_s=0.05)
    robot.connect()
    robot.send_action(_vel(6, 0.1))
    assert _wait_for(lambda: ("speedStop",) in arm.calls)
    assert robot.fault is None
    n = len(arm.calls)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6)))
    time.sleep(0.15)  # a position target is not watched
    assert ("speedStop",) not in arm.calls[n:]
    robot.disconnect()


def test_unitree_zero_velocity_after_a_stale_command(monkeypatch):
    bus = unitree_fake.install(monkeypatch, unitree_fake.low_state([0.1] * 35))
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"), command_timeout_s=0.05)
    robot.connect()
    n = robot.embodiment.num_dofs
    robot.send_action(_vel(n, 0.3))
    sent = len(bus.commands())
    assert _wait_for(lambda: len(bus.commands()) > sent)
    last = bus.commands()[-1]
    slots = robot._motor_index
    assert all(last[s][3] == 0.0 and last[s][5] == 0.0 for s in slots)  # dq, kp
    assert robot.fault is None
    robot.disconnect()
