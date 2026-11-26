"""Embodiment.battery_soc_limit: the type's checks and config parsing.

Temperature limits live per joint (see test_joint_metadata.py); the battery is
one pack for the whole robot, so its ``(warn, shutdown)`` sits on the embodiment.
``warn >= shutdown`` because the warning fires at a *higher* charge than shutdown.
"""

import pytest
import yaml
from pydantic import ValidationError

from hirobot import load_embodiment, make_robot
from hirobot.assembly import RobotAssembly
from hirobot.core.embodiment import Embodiment, Joint

BODY = {"device": "body", "control_hz": 100, "floating_base": True,
        "joints": [{"name": f"j{i}"} for i in range(3)], "driver": {"type": "toy"}}
CAM = {"device": "camera", "width": 32, "height": 24, "driver": {"type": "array"}}


def _bare_body(tmp_path, battery_soc_limit=None):
    """A single-body config; ``make_robot`` returns the driver directly, whose
    embodiment is rebuilt by ``_own_embodiment`` in build.py. That rebuild must carry
    ``battery_soc_limit`` through, or a bare-body robot drops it (the G1 / UR configs
    hiAPP monitors resolve to a bare body)."""
    body = dict(BODY)
    if battery_soc_limit is not None:
        body["battery_soc_limit"] = battery_soc_limit
    (tmp_path / "bare.yaml").write_text(yaml.safe_dump(body))
    return str(tmp_path / "bare.yaml")


def _assembly(tmp_path, battery_soc_limit=None):
    """A one-body + one-camera assembly file; ``make_robot`` returns a RobotAssembly
    whose ``embodiment`` keeps the body's top-level fields."""
    body = dict(BODY)
    if battery_soc_limit is not None:
        body["battery_soc_limit"] = battery_soc_limit
    (tmp_path / "body.yaml").write_text(yaml.safe_dump(body))
    (tmp_path / "cam.yaml").write_text(yaml.safe_dump(CAM))
    (tmp_path / "asm.yaml").write_text(yaml.safe_dump({
        "assembly": {"name": "asm", "parts": {"body": {"from": "./body.yaml"},
                                              "eye": {"from": "./cam.yaml"}},
                     "placement": {}}}))
    return str(tmp_path / "asm.yaml")


def test_battery_soc_limit_defaults_to_none():
    assert Embodiment(name="x").battery_soc_limit is None


def test_battery_soc_limit_accepts_valid():
    emb = Embodiment(name="x", battery_soc_limit=(20.0, 10.0))
    assert emb.battery_soc_limit == (20.0, 10.0)
    # warn == shutdown is allowed (a single hard threshold).
    assert Embodiment(name="x", battery_soc_limit=(10.0, 10.0)).battery_soc_limit == (10.0, 10.0)


@pytest.mark.parametrize("bad", [
    (0.0, 10.0),       # a non-positive tier
    (20.0, 0.0),
    (-5.0, 10.0),
    (110.0, 10.0),     # out of 0-100
    (20.0, 110.0),
    (10.0, 20.0),      # warn < shutdown (reversed)
])
def test_battery_soc_limit_refuses_bad_values(bad):
    with pytest.raises(ValidationError):
        Embodiment(name="x", battery_soc_limit=bad)


def test_config_parses_battery_soc_limit(tmp_path):
    # load_embodiment and an assembly's embodiment both carry the parsed field.
    (tmp_path / "body.yaml").write_text(yaml.safe_dump({**BODY, "battery_soc_limit": [20, 10]}))
    assert load_embodiment(str(tmp_path / "body.yaml")).battery_soc_limit == (20.0, 10.0)
    robot = make_robot(_assembly(tmp_path, battery_soc_limit=[20, 10]))
    assert isinstance(robot, RobotAssembly)
    assert robot.embodiment.battery_soc_limit == (20.0, 10.0)


def test_bare_body_robot_keeps_battery_soc_limit(tmp_path):
    # make_robot(<bare body>) returns the driver, whose embodiment is rebuilt by
    # build._own_embodiment; it must carry battery_soc_limit through so hiAPP can read it.
    robot = make_robot(_bare_body(tmp_path, battery_soc_limit=[20, 10]))
    assert not isinstance(robot, RobotAssembly)
    assert robot.embodiment.battery_soc_limit == (20.0, 10.0)
    assert make_robot(_bare_body(tmp_path)).embodiment.battery_soc_limit is None


def test_config_without_battery_soc_limit_leaves_it_none(tmp_path):
    (tmp_path / "body.yaml").write_text(yaml.safe_dump(BODY))
    assert load_embodiment(str(tmp_path / "body.yaml")).battery_soc_limit is None
    assert make_robot(_assembly(tmp_path)).embodiment.battery_soc_limit is None


def test_config_rejects_reversed_battery_soc_limit(tmp_path):
    (tmp_path / "body.yaml").write_text(yaml.safe_dump({**BODY, "battery_soc_limit": [10, 20]}))
    with pytest.raises(ValueError):
        load_embodiment(str(tmp_path / "body.yaml"))


def test_joints_keep_their_own_temperature_limit_through_config(tmp_path):
    body = {**BODY, "joints": [{"name": "j0", "temperature_limit": [60, 80]},
                               {"name": "j1"}, {"name": "j2"}]}
    (tmp_path / "body.yaml").write_text(yaml.safe_dump(body))
    joints = load_embodiment(str(tmp_path / "body.yaml")).joints
    assert joints[0].temperature_limit == (60.0, 80.0)
    assert joints[1].temperature_limit is None
    # sanity: the type agrees with what the config produced.
    assert Joint(name="j0", temperature_limit=(60, 80)).temperature_limit == (60.0, 80.0)
