"""G1 built-in skills (LocoClient / G1ArmActionClient), faked."""

from __future__ import annotations

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
    assert robot.has_robot_skills
    assert [s.name for s in robot.list_robot_skills()] == [s.name for s in SKILLS]
    with pytest.raises(RobotSkillError, match="connect"):
        robot.run_robot_skill("sit")


def test_unitree_driver_routes_skills_and_refuses_joint_commands():
    robot = _unitree(builtin_skills="g1")
    skills, loco, _ = _skills()
    robot._skills = skills
    assert robot.has_robot_skills and RobotAssembly(robot).has_robot_skills
    result = robot.run_robot_skill("walk", {"vx": 0.3})
    assert result.args == {"vx": 0.3, "vy": 0.0, "vyaw": 0.0, "seconds": 1.0}
    assert loco.calls == [("SetVelocity", 0.3, 0.0, 0.0, 1.0)]
    with pytest.raises(RobotSkillError, match="above"):
        robot.run_robot_skill("walk", {"vx": 2.0})
    assert len(loco.calls) == 1  # the bad call never reached the robot
    robot.reset()  # the built-in service holds the robot: nothing to do
    with pytest.raises(SafetyError, match="built-in motion service owns"):
        robot.send_action(Action(joint_target=np.zeros(robot.embodiment.num_dofs)))

    # Through an assembly too.
    assert RobotAssembly(robot).run_robot_skill("sit").name == "sit"


def test_unitree_driver_without_builtin_skills_has_none():
    robot = _unitree()
    assert not robot.has_robot_skills and robot.list_robot_skills() == []


def test_builtin_skills_excludes_lowcmd_modes():
    emb = load_embodiment(G1)
    for kw in ({"read_only": True}, {"release_motion_mode": True}):
        with pytest.raises(ValueError, match="builtin_skills"):
            get_body("unitree_sdk2")(emb, builtin_skills="g1", **kw)
    with pytest.raises(ValueError, match="'g1' or None"):
        get_body("unitree_sdk2")(emb, builtin_skills="h1")
