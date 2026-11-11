"""MuJoCo driver, and rehearsing from a real robot's pose (examples/rehearse_in_sim.py).

The physics tests need ``mujoco`` and the G1 MJCF from unitree_ros (see
the built-in config unitree_g1_mujoco); they are skipped when either is absent.
The rehearsal test uses a fake real driver, so it never touches hardware.
"""

from __future__ import annotations

import os

import numpy as np
import pytest
import yaml

from hirobot import Action, ControlMode, Observation, Robot, load_embodiment, make_robot
from tests.conftest import mjcf_of

CFG = "unitree_g1_mujoco"
MODEL = mjcf_of(CFG)

mujoco = pytest.importorskip("mujoco")
needs_model = pytest.mark.skipif(not os.path.isfile(MODEL), reason=f"G1 MJCF not found at {MODEL}")


def _sim(fixed_base=True):
    """The G1 in the shared simulation; ``set_state`` and ``_advance`` are on its view."""
    from hirobot.loading.config import load_config

    cfg = load_config(CFG)
    cfg["sim"] = {**cfg["sim"], "realtime": False, "viewer": False,
                  "fixed_base": ["body"] if fixed_base else []}
    if not fixed_base:
        cfg["sim"]["base_height"] = {}
    return _view(make_robot(cfg))


def _view(robot):
    """The MuJoCo view under the gate (and the assembly holding the head camera)."""
    from hirobot.drivers.sim.mujoco.driver import MujocoView

    inner = robot
    while not isinstance(inner, MujocoView):
        inner = getattr(inner, "body", None) or inner._inner
    robot.__dict__["view"] = inner
    return inner


def _hold(robot, target, seconds):
    for _ in range(int(seconds * robot.embodiment.control_hz)):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=target))


@needs_model
def test_joint_names_map_and_fixed_base():
    robot = _sim()
    robot.connect()
    assert robot.model.nq == 29  # free joint removed
    obs = robot.get_observation()
    assert obs.joint_pos.shape == (29,)
    assert np.allclose(obs.base_orientation, [0, 0, 0, 1])


@needs_model
def test_pd_moves_elbow_and_gravity_sags_it():
    """Same move as on hardware: right elbow from hanging (1.35) to horizontal (0.0)."""
    robot = _sim()
    robot.connect()
    names = robot.embodiment.joint_names
    j = names.index("right_elbow")
    start = np.zeros(29)
    start[j] = 1.35
    robot.set_state(start)
    robot.reset()
    target = start.copy()
    target[j] = 0.0
    _hold(robot, target, 3.0)
    obs = robot.get_observation()
    # Reaches the target to within a gravity offset of a few hundredths of a
    # radian (kp 40 against ~1-2 Nm of forearm gravity), on the far side.
    assert 0.0 < obs.joint_pos[j] < 0.1
    assert abs(obs.joint_torque[j]) > 0.3
    others = np.delete(np.arange(29), j)
    assert np.max(np.abs(obs.joint_pos[others] - start[others])) < 0.1


@needs_model
def test_damp_lets_joint_fall_under_gravity():
    robot = _sim()
    robot.connect()
    j = robot.embodiment.joint_names.index("right_elbow")
    start = np.zeros(29)
    robot.set_state(start)  # forearm horizontal
    robot.damp()
    for _ in range(100):
        robot._advance()
    assert robot.get_observation().joint_pos[j] > 0.3  # dropped toward hanging


@needs_model
def test_torque_clamped_to_actuator_range():
    robot = _sim()
    robot.connect()
    j = robot.embodiment.joint_names.index("right_elbow")
    target = np.zeros(29)
    target[j] = 2.0
    robot.send_action(Action(mode=ControlMode.PD, joint_target=target, kp=np.full(29, 1e4),
                             kd=np.zeros(29)))
    lim = robot.model.jnt_actfrcrange[robot.model.actuator_trnid[robot._act[j], 0]][1]
    assert abs(robot.get_observation().joint_torque[j]) <= lim + 1e-6


@needs_model
def test_floating_base_falls():
    robot = _sim(fixed_base=False)
    robot.connect()
    z0 = robot.get_observation().base_position[2]
    robot.damp()
    for _ in range(50):
        robot._advance()
    obs = robot.get_observation()
    assert obs.base_position[2] < z0 - 0.05
    assert obs.base_orientation.shape == (4,)


@needs_model
def test_standing_config_starts_with_feet_on_the_floor():
    from hirobot.loading.config import load_config

    cfg = load_config("unitree_g1_mujoco_standing")
    cfg["sim"]["realtime"] = False
    robot = _view(make_robot(cfg))
    robot.connect()
    assert robot.get_observation().base_position[2] == pytest.approx(0.793, abs=1e-3)
    robot._idle_step()  # one control period with no command
    mj, m, d = robot._mj, robot.model, robot.data
    floor = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "floor")
    touching = {mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
                for c in d.contact[:d.ncon] if floor in (c.geom1, c.geom2)
                for g in (c.geom1, c.geom2) if g != floor}
    assert touching == {"left_ankle_roll_link", "right_ankle_roll_link"}
    robot.disconnect()


@needs_model
@pytest.mark.parametrize("fixed_base", [True, False])
def test_ee_poses_are_wrist_links_in_the_base_frame(fixed_base):
    from hirobot.geometry.kinematics import Kinematics
    from hirobot.geometry.transforms import pose_compose

    robot = _sim(fixed_base=fixed_base)
    robot.connect()
    q = robot.get_observation().joint_pos.copy()
    q[robot.embodiment.joint_names.index("right_elbow")] = 0.8
    _hold(robot, q, 0.5)
    if not fixed_base:
        robot.damp()
        _hold(robot, q, 0.0)
        for _ in range(10):
            robot._advance()  # let it fall and turn, so base frame and world differ
    obs = robot.get_observation()
    assert set(obs.ee_poses) == {"left_hand", "right_hand"}
    kin = Kinematics.from_mjcf(MODEL)
    fk = kin.forward(obs.joint_pos, base_pos=[0, 0, 0], base_quat=[0, 0, 0, 1],
                     joint_names=robot.embodiment.joint_names)  # pelvis = base frame
    for ee in ("left_hand", "right_hand"):
        pos, quat = fk[f"{ee.split('_')[0]}_wrist_yaw_link"]
        pose = obs.ee_poses[ee]
        assert pose.shape == (7,)
        np.testing.assert_allclose(pose[:3], pos, atol=1e-6)
        assert abs(float(np.dot(pose[3:], quat))) == pytest.approx(1.0, abs=1e-6)
    if not fixed_base:
        # world pose = base pose composed with the base-frame pose
        d = robot.data
        bid = robot.model.body("right_wrist_yaw_link").id
        world = pose_compose(np.concatenate([obs.base_position, obs.base_orientation]),
                             obs.ee_poses["right_hand"])
        np.testing.assert_allclose(world[:3], d.xpos[bid], atol=1e-6)
    robot.disconnect()


@needs_model
def test_ee_frame_offset_and_unknown_body():
    def cfg(frame):
        body = {"frames": [{"name": "tool", **frame}], "grippers": []}
        return {"defaults": [CFG], "sim": {"realtime": False},
                "assembly": {"parts": {"body": {"overrides": body}}}}

    robot = _view(make_robot(cfg({"link": "right_wrist_yaw_link",
                                  "pose": [0.1, 0, 0, 0, 0, 0, 1]})))
    robot.connect()
    obs = robot.get_observation()
    d, m = robot.data, robot.model
    pelvis, wrist = m.body("pelvis").id, m.body("right_wrist_yaw_link").id
    expected = d.xpos[wrist] + d.xmat[wrist].reshape(3, 3) @ [0.1, 0.0, 0.0] - d.xpos[pelvis]
    np.testing.assert_allclose(obs.ee_poses["tool"][:3], expected, atol=1e-9)  # pelvis not rotated
    robot.disconnect()

    with pytest.raises(ValueError, match="frame.body 'no_such_link'"):
        make_robot(cfg({"link": "no_such_link"})).connect()


def test_missing_model_reports_path(tmp_path):
    robot = make_robot(CFG, overrides={"body": {"model": {"mjcf": str(tmp_path / "nope.xml")}}})
    with pytest.raises(FileNotFoundError, match="nope.xml"):
        robot.connect()


# --------------------------------------------------------------- pairing


class _FakeReal(Robot):
    """Stands in for UnitreeRobot: records every command it is asked to send."""

    def __init__(self, embodiment, read_only=False, pose=None):
        self._emb = embodiment
        self._read_only = read_only
        self.pose = np.zeros(embodiment.num_dofs) if pose is None else np.asarray(pose, dtype=float)
        self.commands = []
        self.resets = 0
        self.disconnected = False

    @property
    def embodiment(self):
        return self._emb

    def connect(self):
        pass

    def reset(self, seed=None):
        if self._read_only:
            raise RuntimeError("read_only")
        self.resets += 1

    def get_observation(self):
        return Observation(joint_pos=self.pose.copy(), joint_vel=np.zeros_like(self.pose),
                           base_orientation=np.array([0.0, 0, 0, 1.0]))

    def _send_action(self, action):
        if self._read_only:
            raise RuntimeError("read_only")
        self.commands.append(action)

    def disconnect(self):
        self.disconnected = True


def _load_example():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[3] / "examples" / "rehearse_in_sim.py"
    spec = importlib.util.spec_from_file_location("rehearse_in_sim", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@needs_model
def test_rehearsal_starts_from_the_real_pose_and_never_commands_it():
    ex = _load_example()
    emb = load_embodiment(CFG)
    j = emb.joint_names.index("right_elbow")
    pose = np.zeros(29)
    pose[j] = 1.3
    real = _FakeReal(emb, read_only=True, pose=pose)
    real.connect()
    obs = real.get_observation()
    sim = ex.sim_from_pose(CFG, obs, {"realtime": False, "viewer": False})  # sim: overrides
    try:
        assert sim.get_observation().joint_pos[j] == pytest.approx(1.3, abs=0.05)
        q = ex.rehearse(sim, "right_elbow", 0.0, 1.0)
    finally:
        sim.disconnect()
    assert q[j] < 0.3  # the sim did move
    assert real.commands == [] and real.resets == 0


def test_sim_false_builds_the_real_robot_and_sim_true_a_toy_config_in_the_world(tmp_path):
    from hirobot.drivers.sim.mujoco.driver import MujocoView

    real = make_robot(CFG, sim=False)
    assert not isinstance(_inner_body(real), MujocoView)
    path = tmp_path / "g1.yaml"
    path.write_text(yaml.safe_dump({"defaults": ["unitree_g1"]}))
    assert isinstance(_inner_body(make_robot(str(path), sim=True)), MujocoView)


def _inner_body(robot):
    while hasattr(robot, "_inner") or hasattr(robot, "body"):
        robot = getattr(robot, "body", None) or robot._inner
    return robot


def test_floating_base_quaternion_is_scalar_last():
    robot = _sim(fixed_base=False)
    robot.connect()
    # A 90 degree yaw, given scalar-last, must read back unchanged.
    q = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    robot.set_state(robot.get_observation().joint_pos, base_orientation=q)
    assert np.allclose(robot.get_observation().base_orientation, q)
    assert np.allclose(robot.data.qpos[3:7], [q[3], q[0], q[1], q[2]])  # MuJoCo is w first
