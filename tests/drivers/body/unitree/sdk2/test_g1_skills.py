"""G1 built-in skills (LocoClient / G1ArmActionClient), faked."""

from __future__ import annotations

import logging
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import (
    Action,
    RobotAssembly,
    RobotSkillError,
    SafetyError,
    get_body,
    load_embodiment,
)
from hirobot.drivers.body.unitree.sdk2.g1_skills import GESTURES, SKILLS, G1Skills
from hirobot.core.robot_skills import validate_args

G1 = "unitree_g1"


_SPECS = {s.name: s for s in SKILLS}


class _FakeClient:
    """Stands in for both SDK clients; records every call."""

    def __init__(self, code=0):
        self.code, self.calls = code, []

    def SetTimeout(self, t):
        pass

    def Init(self):
        pass

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, *args))
            return self.code

        return call


def _skills(loco=None, arm=None):
    loco = loco or _FakeClient()
    arm = arm or _FakeClient()
    return G1Skills(lambda: loco, lambda: arm), loco, arm


def test_g1_skill_list_is_fixed_and_matches_the_sdk():
    names = [s.name for s in SKILLS]
    assert len(names) == len(set(names))
    assert {"start_locomotion", "walk", "sit", "squat_stand", "high_stand", "damp"} <= set(names)
    assert set(GESTURES) <= set(names) and len(GESTURES) == 15
    # Ids from unitree_sdk2py.g1.arm.g1_arm_action_client.action_map.
    assert GESTURES["shake_hand"][0] == 27 and GESTURES["high_wave"][0] == 26
    assert 99 not in {i for i, _ in GESTURES.values()}  # release arm is stop, not a skill
    assert "zero_torque" not in names
    for spec in SKILLS:
        validate_args(spec, {})  # every default is valid


def test_g1_loco_skills_call_the_sdk():
    skills, loco, _ = _skills()
    skills.run(_SPECS["walk"], {"vx": 0.2, "vy": 0.0, "vyaw": 0.1, "seconds": 2.0})
    skills.run(_SPECS["sit"], {})
    skills.run(_SPECS["start_locomotion"], {})
    skills.run(_SPECS["high_stand"], {})
    skills.run(_SPECS["low_stand"], {})
    assert loco.calls == [("SetVelocity", 0.2, 0.0, 0.1, 2.0), ("SetFsmId", 3),
                          ("SetFsmId", 200), ("SetStandHeight", (1 << 32) - 1),
                          ("SetStandHeight", 0)]


def test_g1_gesture_sends_the_sdk_action_id():
    skills, loco, arm = _skills()
    skills.run(_SPECS["shake_hand"], {})
    skills.run(_SPECS["clap"], {})
    assert arm.calls == [("ExecuteAction", 27), ("ExecuteAction", 17)] and loco.calls == []


def test_g1_errors_say_why():
    skills, _, _ = _skills(arm=_FakeClient(code=3102))
    with pytest.raises(RobotSkillError, match="not running"):
        skills.run(_SPECS["hug"], {})
    skills, _, _ = _skills(loco=_FakeClient(code=7404))
    with pytest.raises(RobotSkillError, match="refused"):
        skills.run(_SPECS["sit"], {})


class _SlowArm(_FakeClient):
    """ExecuteAction replies only when the gesture ends, like the real arm service."""

    def __init__(self, code=0):
        super().__init__(code)
        self.finish = threading.Event()

    def ExecuteAction(self, action_id):
        self.calls.append(("ExecuteAction", action_id))
        self.finish.wait(5)
        return self.code


def test_g1_long_gesture_returns_while_it_runs(caplog):
    arm = _SlowArm(code=3104)  # the RPC times out before a long gesture ends
    skills, _, _ = _skills(arm=arm)
    skills._accept_s = 0.05
    t0 = time.monotonic()
    skills.run(_SPECS["high_wave"], {})  # does not raise, does not wait for the end
    assert time.monotonic() - t0 < 1.0 and arm.calls == [("ExecuteAction", 26)]
    with caplog.at_level(logging.WARNING):
        arm.finish.set()
        for _ in range(100):
            if "high_wave" in caplog.text:
                break
            time.sleep(0.01)
    assert "3104" in caplog.text  # a late failure is only logged


def test_g1_early_gesture_error_still_raises():
    skills, _, _ = _skills(arm=_FakeClient(code=7404))  # wrong FSM: the service answers at once
    with pytest.raises(RobotSkillError, match="7404"):
        skills.run(_SPECS["high_wave"], {})


def test_g1_stop_tries_every_path():
    class _Broken(_FakeClient):
        def StopMove(self):
            raise RuntimeError("no service")

    skills, _, arm = _skills(loco=_Broken())
    skills.stop()
    assert arm.calls == [("ExecuteAction", 99)]


def _unitree(**kw):
    robot = get_body("unitree_sdk2")(load_embodiment(G1), msg_family="hg", **kw)
    robot._connected = True
    robot._low_state = SimpleNamespace(motor_state=[])
    return robot


def test_unitree_driver_lists_skills_without_a_robot():
    robot = get_body("unitree_sdk2")(load_embodiment(G1), builtin_skills="g1")  # not connected
    assert robot.has_onboard_skills
    assert [s.name for s in robot.list_onboard_skills()] == [s.name for s in SKILLS]
    with pytest.raises(RobotSkillError, match="connect"):
        robot.run_onboard_skill("sit")


def test_unitree_driver_routes_skills_and_refuses_joint_commands():
    robot = _unitree(builtin_skills="g1")
    skills, loco, _ = _skills()
    robot._skills = skills
    assert robot.has_onboard_skills and RobotAssembly(robot).has_onboard_skills
    result = robot.run_onboard_skill("walk", {"vx": 0.3})
    assert result.args == {"vx": 0.3, "vy": 0.0, "vyaw": 0.0, "seconds": 1.0}
    assert loco.calls == [("SetVelocity", 0.3, 0.0, 0.0, 1.0)]
    with pytest.raises(RobotSkillError, match="above"):
        robot.run_onboard_skill("walk", {"vx": 2.0})
    assert len(loco.calls) == 1  # the bad call never reached the robot
    robot.reset()  # the built-in service holds the robot: nothing to do
    with pytest.raises(SafetyError, match="built-in motion service owns"):
        robot.send_action(Action(joint_target=np.zeros(robot.embodiment.num_dofs)))

    # Through an assembly too.
    assert RobotAssembly(robot).run_onboard_skill("sit").name == "sit"


def test_unitree_driver_without_builtin_skills_has_none():
    robot = _unitree()
    assert not robot.has_onboard_skills and robot.list_onboard_skills() == []


def test_builtin_skills_excludes_lowcmd_modes():
    emb = load_embodiment(G1)
    for kw in ({"read_only": True}, {"release_motion_mode": True}):
        with pytest.raises(ValueError, match="builtin_skills"):
            get_body("unitree_sdk2")(emb, builtin_skills="g1", **kw)
    with pytest.raises(ValueError, match="'g1' or None"):
        get_body("unitree_sdk2")(emb, builtin_skills="h1")


class _TurningLoco(_FakeClient):
    """A loco client whose body yaw follows the commanded turn rate (scaled by ``gain``)."""

    def __init__(self, yaw0=0.0, gain=1.0, code=0):
        super().__init__(code)
        self.yaw, self.gain, self.vyaw, self.stopped = yaw0, gain, 0.0, threading.Event()
        self._t = time.monotonic()
        self._lock = threading.Lock()

    def read_yaw(self):
        with self._lock:
            now = time.monotonic()
            self.yaw += self.gain * self.vyaw * (now - self._t)
            self._t = now
            return math.remainder(self.yaw, math.tau)  # the IMU wraps at +-pi

    def SetVelocity(self, vx, vy, vyaw, seconds):
        self.read_yaw()
        self.calls.append(("SetVelocity", vx, vy, vyaw, seconds))
        self.vyaw = vyaw
        return self.code

    def StopMove(self):
        self.read_yaw()
        self.calls.append(("StopMove",))
        self.vyaw = 0.0
        self.stopped.set()
        return 0


def _turning(loco, **kw):
    arm = _FakeClient()
    skills = G1Skills(lambda: loco, lambda: arm, yaw=loco.read_yaw, turn_period=0.01, **kw)
    return skills


@pytest.mark.parametrize("degrees,yaw0,gain", [(30.0, 0.0, 1.0), (-45.0, 0.0, 0.7),
                                               (40.0, math.pi - 0.2, 1.0)])
def test_g1_turn_closes_the_loop_on_the_imu(degrees, yaw0, gain):
    # gain 0.7: the robot turns slower than asked, as walk did on the real G1;
    # yaw0 near pi: the IMU wraps to -pi during the turn.
    loco = _TurningLoco(yaw0=yaw0, gain=gain)
    skills = _turning(loco)
    t0 = time.monotonic()
    skills.run(_SPECS["turn"], {"degrees": degrees})
    assert time.monotonic() - t0 < 0.5  # returns while the robot turns
    assert loco.stopped.wait(10.0)
    turned = math.degrees(loco.yaw - yaw0)
    assert abs(turned - degrees) < 3.0
    rates = [c[3] for c in loco.calls if c[0] == "SetVelocity"]
    assert all(math.copysign(1, r) == math.copysign(1, degrees) for r in rates)
    assert max(abs(r) for r in rates) <= 0.6 and all(c[1:3] == (0.0, 0.0) for c in loco.calls
                                                      if c[0] == "SetVelocity")


def test_g1_turn_stop_and_new_skill_end_it():
    loco = _TurningLoco(gain=0.05)  # far too slow to arrive
    skills = _turning(loco)
    skills.run(_SPECS["turn"], {"degrees": 90.0})
    time.sleep(0.05)
    skills.stop()
    assert loco.stopped.is_set() and skills._turn_thread is None
    n = len(loco.calls)
    time.sleep(0.05)
    assert len(loco.calls) == n  # the loop sends nothing after stop

    loco.stopped.clear()
    skills.run(_SPECS["turn"], {"degrees": 90.0})
    skills.run(_SPECS["sit"], {})
    assert loco.stopped.is_set() and loco.calls[-1] == ("SetFsmId", 3)


def test_g1_turn_gives_up_and_stops(caplog):
    loco = _TurningLoco(gain=0.0)  # the robot does not turn at all
    skills = _turning(loco, turn_timeout=0.1)
    with caplog.at_level(logging.WARNING):
        skills.run(_SPECS["turn"], {"degrees": 30.0})
        assert loco.stopped.wait(2.0)
        for _ in range(100):
            if "timed out" in caplog.text:
                break
            time.sleep(0.01)
    assert "timed out" in caplog.text


def test_g1_turn_refusals():
    loco = _TurningLoco(code=7404)  # wrong FSM: the first command is refused
    with pytest.raises(RobotSkillError, match="7404"):
        _turning(loco).run(_SPECS["turn"], {"degrees": 30.0})
    with pytest.raises(RobotSkillError, match="IMU"):
        G1Skills(lambda: _FakeClient(), lambda: _FakeClient()).run(_SPECS["turn"], {"degrees": 30.0})
    with pytest.raises(RobotSkillError, match="fresh"):
        G1Skills(lambda: _FakeClient(), lambda: _FakeClient(),
                 yaw=lambda: None).run(_SPECS["turn"], {"degrees": 30.0})
    with pytest.raises(RobotSkillError, match="above"):
        validate_args(_SPECS["turn"], {"degrees": 200.0})
    assert validate_args(_SPECS["turn"], {}) == {"degrees": 30.0}
    loco = _TurningLoco()
    _turning(loco).run(_SPECS["turn"], {"degrees": 1.0})  # within tolerance: nothing sent
    assert loco.calls == []


def test_unitree_driver_gives_turn_the_imu_yaw():
    robot = _unitree(builtin_skills="g1")
    robot._low_state = SimpleNamespace(motor_state=[], imu_state=SimpleNamespace(rpy=[0.0, 0.0, 0.5]))
    robot._low_state_stamp = robot._state_stamps.tick()
    skills = robot._skill_backend()
    assert skills._yaw() == pytest.approx(0.5)
    robot._low_state_stamp = robot._state_stamps.tick(time.monotonic() - 1.0)
    assert skills._yaw() is None  # stale reading


def test_g1_walk_and_turn_report_their_duration():
    # hiOS waits on result.duration_s; walk and turn have no fixed spec duration.
    assert _SPECS["walk"].duration_s is None and _SPECS["turn"].duration_s is None
    robot = _unitree(builtin_skills="g1")
    robot._skills, _, _ = _skills()
    assert robot.run_onboard_skill("walk", {"vx": 0.2, "seconds": 2.5}).duration_s == 2.5
    assert RobotAssembly(robot).run_onboard_skill("walk", {"seconds": 4.0}).duration_s == 4.0
    assert robot.run_onboard_skill("sit").duration_s is None

    loco = _TurningLoco()
    robot._skills = _turning(loco)
    left = robot.run_onboard_skill("turn", {"degrees": 90.0}).duration_s
    right = RobotAssembly(robot).run_onboard_skill("turn", {"degrees": -90.0}).duration_s
    assert left == right and 3.0 < left < 15.0
    robot.stop_onboard_skill()
    assert robot._skills.turn_estimate_s(1.0) == 0.0  # inside the tolerance nothing runs
    small, big = robot._skills.turn_estimate_s(10.0), robot._skills.turn_estimate_s(180.0)
    assert 0.0 < small < left < big <= 15.0


@pytest.mark.parametrize("degrees", [10.0, 45.0])
def test_g1_turn_estimate_covers_a_slow_robot(degrees):
    # gain 0.7: as slow as walk was on the real G1; the estimate must still cover it.
    loco = _TurningLoco(gain=0.7)
    skills = _turning(loco)
    t0 = time.monotonic()
    skills.run(_SPECS["turn"], {"degrees": degrees})
    assert loco.stopped.wait(10.0)
    assert time.monotonic() - t0 <= skills.turn_estimate_s(degrees)
