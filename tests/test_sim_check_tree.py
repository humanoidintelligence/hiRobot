"""The simulation check built from the robot's tree (two UR5e arms on toy drivers).

Uses the UR5e MJCF from MuJoCo Menagerie (universal_robots_ur5e/ur5e.xml) at
$HIROBOT_UR5E_MJCF, default ~/.cache/hirobot/mujoco_menagerie/; skipped when it or
mujoco is missing.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from hirobot import Action, ControlMode, make_robot
from hirobot.control.safety import SafetyError

mujoco = pytest.importorskip("mujoco")

UR5E = Path(os.environ.get("HIROBOT_UR5E_MJCF", Path.home() / ".cache/hirobot/mujoco_menagerie"
                           "/universal_robots_ur5e/ur5e.xml")).expanduser()
pytestmark = pytest.mark.skipif(not UR5E.is_file(), reason=f"UR5e MJCF not found at {UR5E}")

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
# Toy drivers stand in for the real arms; the model is only for the check.
MODEL = {"model": {"mjcf": str(UR5E), "kp": [2000, 2000, 2000, 500, 500, 500],
                   "kd": [100, 100, 100, 20, 20, 20], "home": HOME}}


def _dual(gap=0.4, scene=None, **check):
    cfg = {"defaults": ["robots/ur5e_dual"],
           "assembly": {"parts": {"left": {"overrides": {"body": MODEL}},
                                  "right": {"overrides": {"body": MODEL}}},
                        "placement": {"left": {"pose": [0, gap, 0, 0, 0, 0, 1]},
                                      "right": {"pose": [0, -gap, 0, 0, 0, 0, 1]}}},
           "safety": {"max_joint_step": 10.0,
                      "sim_check": {"torque_ratio": 1000, "horizon_steps": 30,  # 1 s
                                    "budget_s": 1.0,
                                    **({"scene": {"mjcf": scene}} if scene else {}), **check}}}
    robot = make_robot(cfg)
    robot.connect()
    # The toy arms start at zero, stretched out and overlapping; the real arms
    # would be at home. Put the toys there without a command (the check would
    # rightly refuse moving out of an overlap that deepens on the way).
    for arm in _toys(robot):
        arm._joint_pos = np.array(HOME)
    return robot


def _toys(robot):
    from hirobot.drivers.sim.toy.driver import ToyRobot

    found, seen, todo = [], set(), [robot]
    while todo:
        r = todo.pop()
        if id(r) in seen:
            continue
        seen.add(id(r))
        if isinstance(r, ToyRobot):
            found.append(r)
            continue
        node = getattr(r, "_node", None)
        todo += list(node.robots.values()) if node is not None else []
        todo += [x for x in (getattr(r, "_inner", None), getattr(r, "body", None)) if x is not None]
    assert len(found) == 2, found
    return found


def _pose(left=None, right=None):
    q = np.array(HOME * 2)
    for k, v in (left or {}).items():
        q[k] = v
    for k, v in (right or {}).items():
        q[6 + k] = v
    return Action(mode=ControlMode.POSITION, joint_target=q)


def test_the_check_world_has_both_arms_where_the_config_puts_them():
    robot = _dual()
    sim = robot.safety.sim_check._sim
    m, d = sim.world.model, sim.world.data
    assert sim.names[:2] == ["left/shoulder_pan", "left/shoulder_lift"]
    np.testing.assert_allclose(d.body("left/base").xpos, [0, 0.4, 0], atol=1e-9)
    np.testing.assert_allclose(d.body("right/base").xpos, [0, -0.4, 0], atol=1e-9)
    assert m.ncam == 0
    assert sim._owner[m.body("left/wrist_3_link").id] == "left/body"
    assert sim._owner[m.body("right/base").id] == "right/body"


def test_an_arm_swung_into_the_other_is_refused_and_nothing_is_sent():
    # In the MJCF's frame (base turned 90 deg about z) pan, lift and elbow at 0
    # stretch an arm out along +y, about 0.82 m: the right arm, 0.8 m from the
    # left, reaches into the left arm's shoulder.
    robot = _dual()
    sent = robot.get_observation().joint_pos.copy()
    with pytest.raises(SafetyError, match="collision between parts left/body and right/body"):
        robot.send_action(_pose(right={0: 0.0, 1: 0.0, 2: 0.0}))
    np.testing.assert_allclose(robot.get_observation().joint_pos, sent)
    stats = robot.get_status().safety["sim_check"]
    assert stats["rejected"] == 1 and "collision between parts" in stats["last_reason"]
    # A small move of either arm passes.
    robot.send_action(_pose(left={0: 0.1}, right={0: -0.1}))


def test_a_table_in_the_scene_stops_an_arm_reaching_down(tmp_path):
    table = tmp_path / "table.xml"
    # Table top at 0.15 m, from x = 0.2 to 0.8 m: below both arms at home.
    table.write_text('<mujoco><worldbody><body name="table" pos="0.5 0 0.1">'
                     '<geom type="box" size="0.3 1.0 0.05"/></body></worldbody></mujoco>')
    robot = _dual(scene=str(table))
    robot.send_action(_pose(left={0: 0.1}))  # at home the table is not in the way
    with pytest.raises(SafetyError, match="hits table in the scene"):
        # Pan -90 deg turns the stretched-out direction from +y to +x, over the table;
        # lift and elbow 0 lay the arm out level at the shoulder's height (0.163 m).
        robot.send_action(_pose(left={0: -1.5708, 1: 0.0, 2: 0.0}))


def _extra_joint():
    """ur5e_dual whose left arm lists a joint ``elbow_x`` that its MJCF does not have.

    ``joints:`` in overrides merges by name, so this adds a 7th joint; the
    gains are one value for every joint.
    """
    left = {"model": {"mjcf": str(UR5E), "kp": 1000, "kd": 50},
            "joints": [{"name": "elbow_x"}]}
    return {"defaults": ["robots/ur5e_dual"],
            "assembly": {"parts": {"left": {"overrides": {"body": left}},
                                   "right": {"overrides": {"body": MODEL}}}}}


def test_a_joint_the_mjcf_lacks_is_an_error_naming_it():
    from hirobot.sim.check_sim import check_sim

    with pytest.raises(ValueError, match="'elbow_x'"):
        check_sim(_extra_joint())
    # On a robot the check keeps the error and refuses every command with it.
    robot = make_robot({**_extra_joint(), "safety": {"sim_check": True}})
    robot.connect()
    hold = robot.get_observation().joint_pos  # 13 joints; a zero step
    with pytest.raises(SafetyError, match="elbow_x"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=hold))


def test_hirobot_check_sim_reports_the_world(capsys, tmp_path):
    import yaml

    from hirobot.cli import main

    cfg = {"defaults": ["robots/ur5e_dual"],
           "assembly": {"parts": {"left": {"overrides": {"body": MODEL}},
                                  "right": {"overrides": {"body": MODEL}}}}}
    path = tmp_path / "dual.yaml"
    path.write_text(yaml.safe_dump(cfg))
    assert main(["check", "--sim", str(path)]) == 0
    out = capsys.readouterr().out
    assert "2 bodies, 12 joints" in out and "left/body" in out and out.endswith("OK\n")
    # The toy driver is a simulator: there is no real robot to compare with.
    assert main(["check", "--sim", "--against-real", str(path)]) == 1
    assert "is a simulator" in capsys.readouterr().err


@pytest.fixture
def real_drivers():
    """Two toy drivers that claim to be real: one with read_only, one without."""
    from hirobot.drivers.sim.toy.driver import ToyRobot
    from hirobot.loading.registry import BODIES, register_body

    @register_body("test_real_ro")
    class RealReadOnly(ToyRobot):
        simulated = False
        opened: list = []

        def __init__(self, embodiment, read_only=False, **kw):
            super().__init__(embodiment, **kw)
            self._joint_pos = np.array(HOME)
            RealReadOnly.opened.append(read_only)

    @register_body("test_real_rw")
    class RealNoReadOnly(ToyRobot):
        simulated = False

    try:
        yield RealReadOnly
    finally:
        BODIES._entries.pop("test_real_ro", None)
        BODIES._entries.pop("test_real_rw", None)


def _real_dual(tmp_path, driver):
    import yaml

    body = {**MODEL, "driver": {"type": driver}}
    cfg = {"defaults": ["robots/ur5e_dual"],
           "assembly": {"parts": {"left": {"overrides": {"body": body}},
                                  "right": {"overrides": {"body": body}}}}}
    path = tmp_path / f"{driver}.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return str(path)


def test_against_real_refuses_a_driver_without_read_only(real_drivers, capsys, tmp_path):
    from hirobot.cli import main

    assert main(["check", "--sim", "--against-real", _real_dual(tmp_path, "test_real_rw")]) == 1
    assert "has no read_only mode" in capsys.readouterr().err


def test_against_real_opens_read_only_and_compares(real_drivers, tmp_path):
    from hirobot.sim.check_sim import check_sim

    real_drivers.opened.clear()
    report = check_sim(_real_dual(tmp_path, "test_real_ro"), against_real=True)
    assert real_drivers.opened == [True, True]
    # The toy reports no ee_poses: nothing to compare, and no error.
    assert report.ee_errors == {}
    with pytest.raises(ValueError, match="not bodies"):
        check_sim(_real_dual(tmp_path, "test_real_ro"), against_real=True,
                  drivers={"middle/body": {"ip": "x"}})


def test_hirobot_check_sim_fails_on_a_body_without_a_model(capsys):
    from hirobot.cli import main

    assert main(["check", "--sim", "ur5e_dual"]) == 1
    assert "have no model" in capsys.readouterr().err


def test_the_check_needs_a_model_for_every_body():
    cfg = {"defaults": ["robots/ur5e_dual"],
           "assembly": {"parts": {"left": {"overrides": {"body": MODEL}}}},
           "safety": {"sim_check": True}}
    with pytest.raises(ValueError, match=r"\['right/body'\] have no model"):
        make_robot(cfg)
