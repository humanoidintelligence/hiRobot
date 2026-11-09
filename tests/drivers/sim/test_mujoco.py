"""MuJoCo driver and real/sim pairing (TwinRobot).

The physics tests need ``mujoco`` and the G1 MJCF from unitree_ros (see
the built-in config unitree_g1_mujoco); they are skipped when either is absent.
The pairing tests use a fake real driver, so they never touch hardware.
"""

from __future__ import annotations

import os

import numpy as np
import pytest
import yaml

from hirobot import Action, ControlMode, Observation, Robot, load_embodiment, make_robot
from hirobot.config import load_config
from hirobot.twin import TwinRobot

CFG = "unitree_g1_mujoco"
MODEL = os.path.expanduser(load_config(CFG)["body"]["model_path"])

mujoco = pytest.importorskip("mujoco")
needs_model = pytest.mark.skipif(not os.path.isfile(MODEL), reason=f"G1 MJCF not found at {MODEL}")


def _sim(**overrides):
    over = {"realtime": False, "viewer": False, **overrides}
    return make_robot(CFG, body_overrides=over)


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
    robot = make_robot("unitree_g1_mujoco_standing", body_overrides={"realtime": False})
    robot.connect()
    assert robot.get_observation().base_position[2] == pytest.approx(0.793, abs=1e-3)
    robot.idle_step()  # one control period with no command
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
    from hirobot.kinematics import Kinematics
    from hirobot.transforms import pose_compose

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
    cfg = load_config(CFG)
    cfg["body"].update(realtime=False)
    cfg["embodiment"]["end_effectors"] = [
        {"name": "tool", "frame": {"body": "right_wrist_yaw_link", "pos": [0.1, 0.0, 0.0]}}]
    robot = make_robot(cfg)
    robot.connect()
    obs = robot.get_observation()
    d, m = robot.data, robot.model
    pelvis, wrist = m.body("pelvis").id, m.body("right_wrist_yaw_link").id
    expected = d.xpos[wrist] + d.xmat[wrist].reshape(3, 3) @ [0.1, 0.0, 0.0] - d.xpos[pelvis]
    np.testing.assert_allclose(obs.ee_poses["tool"][:3], expected, atol=1e-9)  # pelvis not rotated
    robot.disconnect()

    cfg["embodiment"]["end_effectors"] = [{"name": "tool", "frame": {"body": "no_such_link"}}]
    with pytest.raises(ValueError, match="frame.body 'no_such_link'"):
        make_robot(cfg).connect()


def test_missing_model_reports_path(tmp_path):
    robot = make_robot(CFG, body_overrides={"model_path": str(tmp_path / "nope.xml")})
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


def _twin(**kw):
    from hirobot.drivers.sim.mujoco import MujocoRobot

    emb = load_embodiment(CFG)
    return MujocoRobot(emb, model_path=MODEL, fixed_base=True, base_height=1.0, realtime=False,
                       default_kp=40.0, default_kd=2.0, **kw)


@needs_model
def test_shadow_never_commands_real():
    emb = load_embodiment(CFG)
    j = emb.joint_names.index("right_elbow")
    pose = np.zeros(29)
    pose[j] = 1.3
    real = _FakeReal(emb, read_only=True, pose=pose)
    robot = TwinRobot(real, _twin(), mode="shadow")
    robot.connect()
    # Sim starts at the real robot's measured pose.
    assert robot.get_observation().joint_pos[j] == pytest.approx(1.3)
    robot.reset()
    target = pose.copy()
    target[j] = 0.0
    for _ in range(100):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=target))
    robot.set_gains(np.full(29, 50.0), None)
    robot.disconnect()
    assert real.commands == [] and real.resets == 0
    assert robot.sim.get_observation().joint_pos[j] < 0.2  # the sim did move


def test_shadow_refuses_writable_real():
    emb = load_embodiment(CFG)
    robot = TwinRobot(_FakeReal(emb, read_only=False), _twin(), mode="shadow")
    with pytest.raises(RuntimeError, match="read_only"):
        robot.connect()


@needs_model
def test_mirror_commands_real_and_renders_its_state():
    emb = load_embodiment(CFG)
    j = emb.joint_names.index("right_elbow")
    real = _FakeReal(emb)
    robot = TwinRobot(real, _twin(), mode="mirror", mirror_hz=200)
    robot.connect()
    robot.reset()
    target = np.zeros(29)
    robot.send_action(Action(mode=ControlMode.PD, joint_target=target))
    assert real.resets == 1 and len(real.commands) == 1
    real.pose[j] = 0.7  # the hardware moved
    import time

    deadline = time.time() + 2.0
    while time.time() < deadline and abs(robot.sim.data.qpos[robot.sim._qadr[j]] - 0.7) > 1e-9:
        time.sleep(0.01)
    assert robot.sim.data.qpos[robot.sim._qadr[j]] == pytest.approx(0.7)
    assert robot.get_observation().joint_pos[j] == pytest.approx(0.7)  # read from real
    robot.disconnect()
    assert real.disconnected and robot.mirror_error is None


def test_safety_gate_applies_through_wrapper():
    from hirobot import SafetyError, SafetyGate

    emb = load_embodiment(CFG)
    real = _FakeReal(emb)
    robot = TwinRobot(real, _twin(), mode="mirror")
    robot.set_safety(SafetyGate.from_embodiment(emb, joint_limit=1.0))
    with pytest.raises(SafetyError):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=np.full(29, 2.0)))
    assert real.commands == []


def test_make_robot_sim_modes(tmp_path):
    cfg = load_config(CFG)
    cfg["body"] = {"driver": "unitree_sdk2", "network_interface": "eth9", "default_kp": 40.0}
    cfg["sim"] = {"mode": False, "model": MODEL}  # bare `off` in YAML
    path = tmp_path / "g1.yaml"
    path.write_text(yaml.safe_dump(cfg))

    # The G1 config carries the head camera and audio, so the robot comes
    # wrapped in a RobotAssembly; the body underneath is what changes.
    off = make_robot(str(path))
    assert type(off.body).__name__ == "UnitreeRobot"
    assert set(off.cameras) == {"head"}

    shadow = make_robot(str(path), sim_overrides={"mode": "shadow"})
    assert isinstance(shadow.body, TwinRobot) and shadow.body.mode == "shadow"
    assert shadow.body.real._read_only and shadow.body.sim._kp[0] == 40.0
    assert shadow.safety is not None

    mirror = make_robot(str(path), sim_overrides={"mode": "mirror"}).body
    assert not mirror.real._read_only
    # The twin renders no cameras even if they carry a mount; images come from the real ones.
    assert all(c.mount is None for c in mirror.sim.embodiment.cameras)

    with pytest.raises(ValueError, match="release_motion_mode"):
        make_robot(str(path), body_overrides={"release_motion_mode": True},
                   sim_overrides={"mode": "shadow"})
    with pytest.raises(ValueError, match="sim.mode"):
        make_robot(str(path), sim_overrides={"mode": "bogus"})


def test_floating_base_quaternion_is_scalar_last():
    robot = _sim(fixed_base=False)
    robot.connect()
    # A 90 degree yaw, given scalar-last, must read back unchanged.
    q = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    robot.set_state(robot.get_observation().joint_pos, base_orientation=q)
    assert np.allclose(robot.get_observation().base_orientation, q)
    assert np.allclose(robot.data.qpos[3:7], [q[3], q[0], q[1], q[2]])  # MuJoCo is w first
