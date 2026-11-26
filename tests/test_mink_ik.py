"""The ``mink_ik`` layer: end-effector targets to joint targets, on a small MuJoCo arm.

The arm's MJCF is written to ``tmp_path``, so nothing depends on downloaded
robot models. Tests that solve IK skip without mink; the last ones check that
hiRobot imports and registers the layer without it.
"""

import subprocess
import sys

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, make_robot
from hirobot.loading.registry import available_layers, load_drivers

# A six-joint arm (yaw, pitch, pitch, roll, pitch, roll) and a tool site; no gravity,
# no contacts, so the PD tracks the targets closely. {free} puts a free joint on
# the base.
ARM_XML = """
<mujoco>
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.002"/>
  <default><joint damping="0.5" armature="0.01" range="-2.8 2.8"/>
    <geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      {free}
      <geom type="cylinder" size="0.05 0.02" mass="2"/>
      <body name="link1" pos="0 0 0.05">
        <joint name="j1" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.03" mass="0.5"/>
        <body name="link2" pos="0 0 0.1">
          <joint name="j2" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.025" mass="0.5"/>
          <body name="link3" pos="0.3 0 0">
            <joint name="j3" axis="0 1 0"/>
            <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.02" mass="0.3"/>
            <body name="link4" pos="0.25 0 0">
              <joint name="j4" axis="1 0 0"/>
              <geom type="capsule" fromto="0 0 0 0.04 0 0" size="0.015" mass="0.1"/>
              <body name="link5" pos="0.04 0 0">
                <joint name="j5" axis="0 1 0"/>
                <geom type="capsule" fromto="0 0 0 0.03 0 0" size="0.015" mass="0.05"/>
                <body name="link6" pos="0.03 0 0">
                  <joint name="j6" axis="1 0 0"/>
                  <geom type="capsule" fromto="0 0 0 0.03 0 0" size="0.015" mass="0.05"/>
                  <site name="tool" pos="0.03 0 0"/>
                  {finger}
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1"/><motor joint="j2"/><motor joint="j3"/><motor joint="j4"/>
    <motor joint="j5"/><motor joint="j6"/>
    {finger_motor}
  </actuator>
</mujoco>
"""
FINGER = ('<body name="finger" pos="0.03 0 0"><joint name="finger" type="slide" axis="0 1 0" '
          'range="0 0.04"/><geom size="0.01" mass="0.01"/></body>')
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
LAYER = {"frames": {"tcp": {"name": "tool", "type": "site"}}}


def _xml(tmp_path, name="arm.xml", free=False, finger=False):
    path = tmp_path / name
    path.write_text(ARM_XML.format(free="<freejoint/>" if free else "",
                                   finger=FINGER if finger else "",
                                   finger_motor='<motor joint="finger"/>' if finger else ""))
    return str(path)


def _embodiment(free=False, finger=False):
    joints = JOINTS + (["finger"] if finger else [])
    return Embodiment(name="ik-arm", kind="arm", floating_base=free, control_hz=50,
                      joints=[{"name": n} for n in joints],
                      end_effectors=[{"name": "tcp", "frame": {"body": "link6",
                                                               "pos": [0.03, 0.0, 0.0]}}])


def _sim(tmp_path, free=False, finger=False):
    from hirobot.drivers.sim.mujoco.driver import MujocoRobot

    return MujocoRobot(_embodiment(free, finger), model_path=_xml(tmp_path, free=free,
                                                                 finger=finger),
                       realtime=False, ramp_seconds=0.0, default_kp=200.0, default_kd=5.0,
                       initial_joint_pos=[0.0, -0.4, 0.8, 0.0, -0.4, 0.0] + ([0.0] if finger else []))


@pytest.fixture
def mink():
    return pytest.importorskip("mink")


def _layer(tmp_path, free=False, finger=False, **config):
    from hirobot.layers.mink_ik import MinkIkLayer

    layer = MinkIkLayer(_sim(tmp_path, free, finger), **{**LAYER, **config})
    layer.connect()
    layer.reset()
    return layer


def _run(layer, action, steps):
    for _ in range(steps):
        layer.send_action(action)


def _cart(pose):
    return Action(mode=ControlMode.CARTESIAN, ee_targets={"tcp": np.asarray(pose, dtype=float)})


def _delta(d):
    return Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={"tcp": np.asarray(d, dtype=float)})


def test_capabilities(tmp_path, mink):
    layer = _layer(tmp_path)
    modes = layer.action_modes
    assert ControlMode.CARTESIAN in modes and ControlMode.CARTESIAN_DELTA in modes
    assert ControlMode.POSITION in modes  # the robot's own modes pass through
    assert layer.cartesian_targets == ("tcp",)
    assert layer.direct_joints == ()


def test_cartesian_reaches_a_pose(tmp_path, mink):
    layer = _layer(tmp_path)
    start = layer.get_observation().ee_poses["tcp"]
    goal = start.copy()
    goal[:3] += [-0.08, 0.10, 0.06]
    _run(layer, _cart(goal), 100)  # 2 s
    obs = layer.get_observation()
    assert np.linalg.norm(obs.ee_poses["tcp"][:3] - goal[:3]) < 0.01
    assert np.linalg.norm(layer.commanded_ee_poses["tcp"][:3] - goal[:3]) < 0.01
    assert layer.stats["count"] == 100 and layer.stats["failures"] == 0


def test_cartesian_delta_integrates_against_the_target(tmp_path, mink):
    layer = _layer(tmp_path)
    start = layer.get_observation().ee_poses["tcp"]
    _run(layer, _delta([0.001, 0.0, 0.001, 0.0, 0.0, 0.0]), 50)  # 5 cm along x and z
    _run(layer, Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}), 25)  # settle
    moved = layer.get_observation().ee_poses["tcp"][:3] - start[:3]
    np.testing.assert_allclose(moved, [0.05, 0.0, 0.05], atol=0.01)
    # A rotation delta (axis-angle in the base frame) turns the target.
    before = layer.commanded_ee_poses["tcp"][3:]
    _run(layer, _delta([0, 0, 0, 0.0, 0.0, 0.01]), 20)  # 0.2 rad about z
    after = layer.commanded_ee_poses["tcp"][3:]
    angle = 2 * np.arccos(min(1.0, abs(float(np.dot(before, after)))))
    assert angle == pytest.approx(0.2, abs=0.05)


def test_default_frames_apply_the_end_effector_offset(tmp_path, mink):
    """frames: left out: the target is the end-effector frame (link plus offset), not the link."""
    from hirobot.drivers.sim.mujoco.driver import MujocoRobot
    from hirobot.layers.mink_ik import MinkIkLayer

    # A TCP 8 cm along link6's x and 3 cm along its z, turned 90 degrees about z.
    s = np.sqrt(0.5)
    emb = Embodiment(name="ik-arm", kind="arm", control_hz=50,
                     joints=[{"name": n} for n in JOINTS],
                     end_effectors=[{"name": "tcp", "frame": {
                         "body": "link6", "pos": [0.08, 0.0, 0.03], "quat": [0.0, 0.0, s, s]}}])
    sim = MujocoRobot(emb, model_path=_xml(tmp_path), realtime=False, ramp_seconds=0.0,
                      default_kp=200.0, default_kd=5.0,
                      initial_joint_pos=[0.0, -0.4, 0.8, 0.0, -0.4, 0.0])
    layer = MinkIkLayer(sim)  # no frames: from the embodiment, link6 plus the offset
    layer.connect()
    layer.reset()
    start = layer.get_observation().ee_poses["tcp"]
    # The layer reports the same TCP pose as the simulator.
    np.testing.assert_allclose(start, sim.get_observation().ee_poses["tcp"], atol=1e-6)

    # (a) Empty deltas hold the TCP where it is.
    _run(layer, Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}), 50)
    here = layer.get_observation().ee_poses["tcp"]
    assert np.linalg.norm(here[:3] - start[:3]) < 0.002
    assert abs(float(np.dot(here[3:], start[3:]))) > 0.9999

    # (b) A CARTESIAN target is reached at the TCP, not at the link.
    goal = start.copy()
    goal[:3] += [-0.06, 0.08, 0.05]
    _run(layer, _cart(goal), 100)
    obs = layer.get_observation()
    assert np.linalg.norm(obs.ee_poses["tcp"][:3] - goal[:3]) < 0.01
    assert abs(float(np.dot(obs.ee_poses["tcp"][3:], goal[3:]))) > 0.999
    np.testing.assert_allclose(obs.ee_poses["tcp"], sim.get_observation().ee_poses["tcp"],
                               atol=1e-6)


def test_ee_poses_match_the_simulation(tmp_path, mink):
    from hirobot.layers.mink_ik import MinkIkLayer

    sim = _sim(tmp_path)
    layer = MinkIkLayer(sim, **LAYER)
    layer.connect()
    layer.reset()
    _run(layer, _delta([0.002, -0.001, 0.0, 0, 0, 0]), 10)
    # The layer's FK from the measured joints and the simulator's frame (link6
    # plus offset) are the same point; the IK frame is the site there.
    ours = layer.get_observation().ee_poses["tcp"]
    np.testing.assert_allclose(ours[:3], sim.get_observation().ee_poses["tcp"][:3], atol=1e-6)


def test_other_modes_pass_through(tmp_path, mink):
    layer = _layer(tmp_path)
    target = np.array([0.3, -0.2, 0.5, 0.1, -0.3, 0.2])
    _run(layer, Action(mode=ControlMode.POSITION, joint_target=target), 50)
    np.testing.assert_allclose(layer.get_observation().joint_pos, target, atol=0.01)
    assert layer.stats["count"] == 0  # no IK ran
    # The next IK command starts from where the robot is now, not the old target.
    pose = layer.get_observation().ee_poses["tcp"]
    _run(layer, Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}), 10)
    np.testing.assert_allclose(layer.get_observation().ee_poses["tcp"][:3], pose[:3], atol=0.005)


def test_hold_reseeds_from_the_measured_pose(tmp_path, mink):
    layer = _layer(tmp_path)
    start = layer.get_observation().ee_poses["tcp"]
    _run(layer, _delta([0.002, 0.0, 0.0, 0, 0, 0]), 25)  # the target is 5 cm ahead
    layer.hold()
    here = layer.get_observation().ee_poses["tcp"]
    _run(layer, Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}), 25)
    after = layer.get_observation().ee_poses["tcp"]
    # It stays where hold() left it; it does not run on to the old target.
    np.testing.assert_allclose(after[:3], here[:3], atol=0.005)
    assert np.linalg.norm(here[:3] - start[:3]) > 0.02


def test_unreachable_target_does_not_raise(tmp_path, mink):
    layer = _layer(tmp_path)
    _run(layer, _cart([3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), 60)  # 3 m: the arm reaches 0.65 m
    obs = layer.get_observation()
    assert np.all(np.isfinite(obs.joint_pos))
    assert np.all(np.abs(obs.joint_pos) <= 2.8 + 0.05)  # inside the joint ranges
    assert 0.5 < obs.ee_poses["tcp"][0] < 0.7  # stretched out towards it


def test_solver_failure_holds_the_previous_target(tmp_path, mink, monkeypatch, caplog):
    layer = _layer(tmp_path)
    _run(layer, _delta([0.001, 0, 0, 0, 0, 0]), 5)
    before = layer.get_observation().joint_pos

    def broken(*args, **kwargs):
        raise RuntimeError("no solution")

    monkeypatch.setattr(mink, "solve_ik", broken)
    with caplog.at_level("WARNING", logger="hirobot.layers.mink_ik"):
        _run(layer, _delta([0.01, 0, 0, 0, 0, 0]), 10)
    assert layer.stats["failures"] == 10 and "holding the previous target" in caplog.text
    np.testing.assert_allclose(layer.get_observation().joint_pos, before, atol=0.01)


def test_commanded_joint_targets(tmp_path, mink, monkeypatch):
    from hirobot.layers.mink_ik import MinkIkLayer

    sim = _sim(tmp_path)
    sent = []
    send = sim.send_action
    monkeypatch.setattr(sim, "send_action", lambda a: (sent.append(a), send(a))[1])
    layer = MinkIkLayer(sim, **LAYER)
    layer.connect()
    layer.reset()
    assert layer.commanded_joint_targets is None  # no IK command yet

    _run(layer, _delta([0.002, 0.0, 0.001, 0, 0, 0]), 3)
    down = sent[-1]
    assert down.mode == ControlMode.POSITION
    got = layer.commanded_joint_targets
    assert set(got) == set(JOINTS)
    names = down.joint_names or sim.embodiment.joint_names
    for name, value in zip(names, down.joint_target):
        assert got[name] == pytest.approx(float(value))

    # A copy: changing it leaves the layer alone.
    got["j1"] = 99.0
    assert layer.commanded_joint_targets["j1"] != 99.0

    # A solver failure resends the previous target, and the property keeps it.
    before = layer.commanded_joint_targets
    monkeypatch.setattr(mink, "solve_ik", lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("no solution")))
    _run(layer, _delta([0.01, 0, 0, 0, 0, 0]), 2)
    assert layer.stats["failures"] == 2
    assert layer.commanded_joint_targets == before
    np.testing.assert_allclose(sent[-1].joint_target, [before[n] for n in JOINTS])
    monkeypatch.undo()  # the real solver (and sim.send_action) again

    layer.hold()
    assert layer.commanded_joint_targets is None
    _run(layer, _delta(np.zeros(6)), 1)
    assert layer.commanded_joint_targets is not None
    layer.reset()
    assert layer.commanded_joint_targets is None
    _run(layer, _delta(np.zeros(6)), 1)
    layer.send_action(Action(mode=ControlMode.POSITION,
                             joint_target=layer.get_observation().joint_pos))
    assert layer.commanded_joint_targets is None  # passed through: not the IK's target


def test_bad_targets_are_refused(tmp_path, mink):
    from hirobot.control.safety import SafetyError

    layer = _layer(tmp_path)
    with pytest.raises(SafetyError, match="no IK frame"):
        layer.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"nope": np.zeros(7)}))
    with pytest.raises(SafetyError, match="must be"):
        layer.send_action(_cart(np.zeros(6)))
    with pytest.raises(SafetyError, match="must be"):
        layer.send_action(_delta(np.zeros(7)))


def test_floating_base_is_frozen(tmp_path, mink):
    layer = _layer(tmp_path, free=True)
    q0 = layer._ik["model"].qpos0[:7].copy()
    start = layer.get_observation().ee_poses["tcp"]
    goal = start.copy()
    goal[:3] += [-0.05, 0.05, 0.05]
    _run(layer, _cart(goal), 60)
    np.testing.assert_allclose(layer._q[:7], q0, atol=1e-9)  # the IK never moved the base
    # Targets are in the base frame: the arm reached it relative to its base.
    assert np.linalg.norm(layer.commanded_ee_poses["tcp"][:3] - goal[:3]) < 0.01


def test_joints_outside_the_ik_model_are_free(tmp_path, mink):
    from hirobot.layers.mink_ik import MinkIkLayer

    sim = _sim(tmp_path, finger=True)  # the simulation has a finger joint...
    ik_model = _xml(tmp_path, "ik.xml")  # ...the IK model does not
    layer = MinkIkLayer(sim, model=ik_model, **LAYER)
    layer.connect()
    layer.reset()
    assert layer.direct_joints == ("finger",)
    action = _delta(np.zeros(6))
    action.joint_names, action.joint_target = ["finger"], np.array([0.03])
    _run(layer, action, 50)
    assert layer.get_observation().joint_pos[6] == pytest.approx(0.03, abs=0.005)


def test_uses_only_the_public_api(tmp_path, mink):
    from hirobot.layers.mink_ik import MinkIkLayer
    from hirobot.testing import assert_layer_uses_public_api

    actions = [_delta([0.001, 0, 0, 0, 0, 0]), _cart([0.4, 0.1, 0.3, 0, 0, 0, 1]),
               Action(mode=ControlMode.POSITION, joint_target=np.zeros(6))]
    used = assert_layer_uses_public_api(MinkIkLayer, LAYER, inner=_sim(tmp_path),
                                        actions=actions)
    assert {"send_action", "get_observation", "model"} <= used


def test_make_robot_with_a_layers_entry(tmp_path, mink):
    from tests.conftest import toy_config

    cfg = toy_config(_embodiment(), driver={"type": "mujoco", "model_path": _xml(tmp_path),
                                            "realtime": False, "ramp_seconds": 0.0,
                                            "default_kp": 200.0, "default_kd": 5.0},
                     layers=[{"type": "mink_ik", "model": _xml(tmp_path, "ik.xml"),
                              "frames": {"tcp": {"name": "tool", "type": "site"}}}])
    robot = make_robot(cfg)
    assert type(robot).__name__ == "MinkIkLayer"
    robot.connect()
    try:
        robot.reset()
        start = robot.get_observation().ee_poses["tcp"]
        _run(robot, _delta([0.0, 0.001, 0.0, 0, 0, 0]), 40)  # through the top gate
        moved = robot.get_observation().ee_poses["tcp"][:3] - start[:3]
        assert moved[1] == pytest.approx(0.04, abs=0.01)
    finally:
        robot.disconnect()


def test_model_from_a_config_name(mink):
    from hirobot.layers.mink_ik import resolve_model

    assert resolve_model("unitree_g1").endswith("g1_29dof_rev_1_0.xml")
    assert resolve_model("/x/arm.xml") == "/x/arm.xml"
    with pytest.raises(ValueError, match="model"):
        resolve_model("")


def _body_file(tmp_path, name, model):
    path = tmp_path / name
    path.write_text("device: body\ncontrol_hz: 50\nfloating_base: false\n"
                    f"joints: [{{name: j1}}]\nmodel: {model}\n")
    return str(path)


def test_model_from_a_config_file(tmp_path):
    from hirobot.layers.mink_ik import resolve_model

    mjcf = str(tmp_path / "arm.xml")
    for name in ("arm.yaml", "arm.yml"):
        assert resolve_model(_body_file(tmp_path, name, f"{{mjcf: {mjcf}}}")) == mjcf
    urdf_only = _body_file(tmp_path, "urdf.yaml", f"{{urdf: {tmp_path / 'arm.urdf'}}}")
    with pytest.raises(ValueError, match="has only a urdf; mink_ik needs an mjcf"):
        resolve_model(urdf_only)
    xml = tmp_path / "ik.xml"
    xml.write_text("<mujoco/>")
    assert resolve_model(str(xml)) == str(xml)  # an existing .xml is the MJCF itself


def test_a_real_robot_needs_model(mink):
    from hirobot.drivers.sim.toy import ToyRobot
    from hirobot.layers.mink_ik import MinkIkLayer

    layer = MinkIkLayer(ToyRobot(_embodiment()))
    with pytest.raises(ValueError, match="give model:"):
        layer.connect()


# ------------------------------------------------------- without mink


def test_registered_with_the_builtins():
    load_drivers()
    assert "mink_ik" in available_layers()


def test_import_and_registry_work_without_mink(tmp_path):
    # A fresh interpreter where mink cannot be imported.
    code = (
        "import sys; sys.modules['mink'] = None\n"
        "import hirobot\n"
        "from hirobot.loading.registry import available_layers, get_layer, load_drivers\n"
        "load_drivers()\n"
        "assert 'mink_ik' in available_layers()\n"
        "from hirobot.drivers.sim.toy import ToyRobot\n"
        "from hirobot import Embodiment\n"
        "emb = Embodiment(name='a', joints=[{'name': 'j'}], control_hz=50,\n"
        "                 end_effectors=[{'name': 'tcp', 'frame': {'body': 'b'}}])\n"
        "try:\n"
        "    get_layer('mink_ik')(ToyRobot(emb), model='arm.xml')\n"
        "except ImportError as exc:\n"
        "    print(exc)\n"
    )
    import os
    from pathlib import Path

    import hirobot

    src = str(Path(hirobot.__file__).resolve().parents[1])  # the hirobot under test
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(
        [src, os.environ.get("PYTHONPATH", "")]).rstrip(os.pathsep)}
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                          cwd=tmp_path, env=env, timeout=120)
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "pip install 'hirobot[ik]'" in proc.stdout
