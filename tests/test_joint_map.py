"""A hand's ``joint_map`` (one hand joint drives several model joints), the model's
``joint_names`` rename, and the hold on actuators no view drives.

The MuJoCo tests use a small MJCF written here (no gravity): an arm joint ``b``,
two finger joints ``a1`` / ``a2`` the hand joint ``h`` maps to, and a joint ``w``
with a motor the embodiment does not list (like H1's wrist roll). Collisions are
off: the tests are about the mapping.
"""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import Action, ControlMode, load_embodiment, make_robot
from hirobot.loading.tree import load_robot

HAND = {"device": "end_effector", "interface": "joints",
        "joints": [{"name": "h", "position_limit": [0, 1]}],
        "joint_map": {"h": [{"joint": "a1"}, {"joint": "a2", "offset": 0.2, "scale": -0.5}]}}

MJCF = """<mujoco>
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.002"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="base">
      <body name="link">
        <joint name="b" type="hinge" axis="0 0 1" range="-3 3" armature="0.01" damping="0.1"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/>
        <body name="wrist" pos="0.2 0 0">
          <joint name="{wrist}" type="hinge" axis="1 0 0" armature="0.01" damping="0.1"/>
          <geom type="box" size="0.02 0.02 0.02" mass="0.05" pos="0 0 0.03"/>
          <body name="f1" pos="0.03 0 0">
            <joint name="a1" type="hinge" axis="0 1 0" range="-2 2" armature="0.01" damping="0.1"/>
            <geom type="capsule" fromto="0 0 0 0.05 0 0" size="0.01" mass="0.02"/>
          </body>
          <body name="f2" pos="0.03 0.05 0">
            <joint name="a2" type="hinge" axis="0 1 0" range="-2 2" armature="0.01" damping="0.1"/>
            <geom type="capsule" fromto="0 0 0 0.05 0 0" size="0.01" mass="0.02"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="b"/>
    <motor joint="{wrist}"/>
    <motor joint="a1"/>
    <motor joint="a2"/>
  </actuator>
</mujoco>
"""


def _cfg(hand=None, **hand_keys):
    return {"assembly": {"name": "mapped", "parts": {
        "body": {"device": "body", "control_hz": 50, "joints": [{"name": "b"}],
                 "mounts": {"flange": {"link": "wrist"}}, "driver": {"type": "toy"}},
        "hand": {**(hand or HAND), **hand_keys}},
        "placement": {"hand": {"on": "body.flange"}}}}


# ------------------------------------------------------------------ parsing


def test_joint_map_is_parsed_with_defaults():
    spec = load_robot(_cfg())
    assert spec.root.parts["hand"].fields["joint_map"] == {"h": [
        {"joint": "a1", "scale": 1.0, "offset": 0.0},
        {"joint": "a2", "scale": -0.5, "offset": 0.2}]}


@pytest.mark.parametrize("joint_map, match", [
    ({"x": [{"joint": "a1"}]}, r"'x' is not a joint of this hand"),
    ({"h": {"joint": "a1"}}, "non-empty list"),
    ({"h": []}, "non-empty list"),
    ({"h": [{"joint": "a1", "scale": 0}]}, "scale must not be 0"),
    ({"h": [{"joint": "a1", "gain": 2}]}, r"unknown key\(s\) \['gain'\]"),
    ({"h": [{"scale": 1}]}, "model joint name"),
    ({"h": [{"joint": "a1", "offset": float("nan")}]}, "offset must be a finite number"),
    ({"h": [{"joint": "a1"}, {"joint": "a1"}]}, "listed by both"),
    ([{"joint": "a1"}], "must map hand joints"),
])
def test_a_bad_joint_map_is_refused_at_load(joint_map, match):
    with pytest.raises(ValueError, match=match):
        load_robot(_cfg(joint_map=joint_map))


def test_joint_map_on_the_opening_interface_is_refused():
    gripper = {"device": "end_effector", "interface": "opening",
               "joint_map": {"h": [{"joint": "a1"}]}}
    with pytest.raises(ValueError, match="opening interface has no joints"):
        load_robot(_cfg(hand=gripper))


def test_the_embodiment_carries_the_map_prefixed_in_a_nested_node():
    emb = load_embodiment(_cfg())
    (hand,) = [e for e in emb.end_effectors if e.name == "hand"]
    assert [(e.joint, e.scale, e.offset) for e in hand.joint_map["h"]] == [
        ("a1", 1.0, 0.0), ("a2", -0.5, 0.2)]
    outer = {"assembly": {"name": "cell", "parts": {"arm": _cfg()}}}
    emb = load_embodiment(outer)
    (hand,) = [e for e in emb.end_effectors if e.name == "arm/hand"]
    assert list(hand.joint_map) == ["arm/h"]
    assert [e.joint for e in hand.joint_map["arm/h"]] == ["arm/a1", "arm/a2"]


def test_an_end_effector_refuses_a_map_for_a_joint_it_does_not_list():
    from hirobot.core.embodiment import EndEffector

    with pytest.raises(ValueError, match=r"joint_map lists \['x'\]"):
        EndEffector(name="hand", joints=["h"], joint_map={"x": [{"joint": "a1"}]})


@pytest.mark.parametrize("names, match", [
    ({"torso": 3}, "must map model joint names"),
    ({"a": "x", "b": "x"}, "same name"),
])
def test_a_bad_joint_names_rename_is_refused(names, match, tmp_path):
    cfg = _cfg()
    cfg["assembly"]["parts"]["body"]["model"] = {"mjcf": str(tmp_path / "m.xml"),
                                                 "joint_names": names}
    with pytest.raises(ValueError, match=match):
        load_robot(cfg)


# -------------------------------------------------------------- simulation

mujoco = pytest.importorskip("mujoco")


def _model(tmp_path, wrist="w", **model):
    path = tmp_path / "mapped.xml"
    path.write_text(MJCF.replace("{wrist}", wrist))
    return {"mjcf": str(path), **model}


def _mujoco_robot(tmp_path, **model):
    """The robot on the single-model ``mujoco`` driver (no shared world)."""
    cfg = _cfg()
    cfg["assembly"]["parts"]["body"].update(model=_model(tmp_path, **model),
                                            driver={"type": "mujoco", "realtime": False})
    return make_robot(cfg)


def _sim_robot(tmp_path, **model):
    """The robot in a shared world (``sim:``)."""
    cfg = _cfg()
    cfg["assembly"]["parts"]["body"]["model"] = _model(tmp_path, **model)
    cfg["sim"] = {"realtime": False, "ramp_seconds": 0.1}
    return make_robot(cfg)


def _driver(robot):
    from hirobot.drivers.sim.mujoco.driver import MujocoRobot

    todo, seen = [robot], set()
    while todo:
        r = todo.pop()
        if id(r) in seen:
            continue
        seen.add(id(r))
        if isinstance(r, MujocoRobot):
            return r
        node = getattr(r, "_node", None)
        todo += list(node.robots.values()) if node is not None else []
        todo += [x for x in (getattr(r, "_inner", None), getattr(r, "body", None),
                             getattr(r, "_own", None)) if x is not None]
    raise AssertionError("no MuJoCo driver in the robot")


def _q(drv, name):
    m = drv.model
    return float(drv.data.qpos[m.jnt_qposadr[m.joint(name).id]])


@pytest.mark.parametrize("build", [_mujoco_robot, _sim_robot], ids=["mujoco", "sim"])
def test_a_hand_value_drives_every_mapped_joint_and_reads_back(build, tmp_path):
    robot = build(tmp_path, kp=[20, 2], kd=[1, 0.1])
    robot.connect()
    try:
        names = robot.embodiment.joint_names
        assert names == ["b", "h"]
        target = np.array([0.3, 0.6])
        for _ in range(100):  # 2 s
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
        drv = _driver(robot)
        assert _q(drv, "a1") == pytest.approx(0.6, abs=0.01)          # 0 + 1 * 0.6
        assert _q(drv, "a2") == pytest.approx(0.2 - 0.5 * 0.6, abs=0.01)  # -0.1
        obs = robot.get_observation()
        np.testing.assert_allclose(obs.joint_pos, target, atol=0.01)
        assert abs(obs.joint_vel[1]) < 0.05
    finally:
        robot.disconnect()


@pytest.mark.parametrize("build", [_mujoco_robot, _sim_robot], ids=["mujoco", "sim"])
def test_set_state_puts_every_mapped_joint_in_place(build, tmp_path):
    robot = build(tmp_path)
    robot.connect()
    try:
        _driver(robot).set_state(np.array([0.1, 0.4]))
        drv = _driver(robot)
        assert _q(drv, "a1") == pytest.approx(0.4)
        assert _q(drv, "a2") == pytest.approx(0.2 - 0.5 * 0.4)
        np.testing.assert_allclose(robot.get_observation().joint_pos, [0.1, 0.4])
    finally:
        robot.disconnect()


@pytest.mark.parametrize("build", [_mujoco_robot, _sim_robot], ids=["mujoco", "sim"])
def test_joint_names_renames_a_model_joint_before_it_compiles(build, tmp_path):
    # The MJCF calls the wrist "wrist_old"; the rename makes it "w". The embodiment does
    # not list it, so a missing rename would only show as a missing joint below.
    robot = build(tmp_path, wrist="wrist_old", joint_names={"wrist_old": "w"})
    robot.connect()
    try:
        m = _driver(robot).model
        assert m.joint("w").id >= 0
        with pytest.raises(KeyError):
            m.joint("wrist_old")
        # The wrist's actuator followed the rename.
        jid = m.joint("w").id
        assert any(m.actuator_trnid[a, 0] == jid for a in range(m.nu))
    finally:
        robot.disconnect()


def test_joint_names_naming_a_joint_the_model_lacks_is_an_error(tmp_path):
    robot = _sim_robot(tmp_path, joint_names={"torso_joint": "waist_yaw_joint"})
    with pytest.raises(ValueError, match=r"renames \['torso_joint'\], which are not joints"):
        robot.connect()


def test_an_actuator_no_view_drives_holds_its_start_pose(tmp_path):
    robot = _sim_robot(tmp_path)
    robot.connect()
    try:
        drv = _driver(robot)
        m, d = drv.model, drv.data
        w = m.joint("w")
        d.qpos[m.jnt_qposadr[w.id]] = 0.0
        # Swing the arm hard both ways: the wrist hangs on it, so without a hold it swings.
        for k in range(100):
            target = np.array([1.0 if (k // 25) % 2 == 0 else -1.0, 0.5])
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
        assert abs(_q(drv, "w")) < 0.05
        act = next(a for a in range(m.nu) if m.actuator_trnid[a, 0] == w.id)
        assert m.actuator_biasprm[act, 1] < 0  # a stiffness, written once at connect
    finally:
        robot.disconnect()


def test_the_simulation_check_refuses_a_hand_with_a_joint_map(tmp_path):
    cfg = _cfg()
    cfg["assembly"]["parts"]["body"]["model"] = _model(tmp_path)
    cfg["safety"] = {"sim_check": True}
    with pytest.raises(ValueError, match=r"does not support hand joint_map yet \(hand\)"):
        make_robot(cfg)
