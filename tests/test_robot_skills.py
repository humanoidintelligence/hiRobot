"""Onboard skills: the default (none) and argument checks. G1 skills: tests/drivers/body/unitree/."""

from types import SimpleNamespace

import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import (
    RobotSkillError,
    RobotSkillSpec,
    get_body,
    load_embodiment,
)
from hirobot.drivers.body.unitree.sdk2.g1_skills import SKILLS, G1Skills
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


def test_default_robot_has_no_skills():
    robot = get_body("toy")(load_embodiment(G1))
    assert not robot.has_onboard_skills and robot.list_onboard_skills() == []
    with pytest.raises(RobotSkillError, match="unknown onboard skill"):
        robot.run_onboard_skill("walk")
    robot.stop_onboard_skill()  # no-op


def test_validate_args_defaults_ranges_and_types():
    spec = RobotSkillSpec(name="walk", description="", parameters={
        "type": "object",
        "properties": {
            "vx": {"type": "number", "minimum": -0.5, "maximum": 0.5, "default": 0.0},
            "seconds": {"type": "number", "exclusiveMinimum": 0, "default": 1.0},
        },
    })
    assert validate_args(spec, {"vx": 0.2}) == {"vx": 0.2, "seconds": 1.0}
    for bad, msg in (({"vx": 0.9}, "above"), ({"vx": "fast"}, "number"),
                     ({"vx": True}, "number"), ({"vx": float("nan")}, "finite"),
                     ({"seconds": 0}, "above 0"), ({"vy": 1}, "unknown")):
        with pytest.raises(RobotSkillError, match=msg):
            validate_args(spec, bad)


def _unitree(**kw):
    robot = get_body("unitree_sdk2")(load_embodiment(G1), msg_family="hg", **kw)
    robot._connected = True
    robot._low_state = SimpleNamespace(motor_state=[])
    return robot


