"""Hands in the embodiment and RobotAssembly (G1 + Dex3 on a fake SDK), and MuJoCo fingers.

The hand drivers themselves are tested under ``tests/drivers/end_effector/``.
"""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import Action, ControlMode, RobotAssembly, load_embodiment, make_robot
from hirobot.drivers.end_effector import UnitreeDex3Hand
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.devices.end_effector import (
    available_end_effectors,
    body_embodiment,
    build_end_effectors,
    end_effector_joint_names,
    get_end_effector,
)
from tests.drivers.end_effector.unitree.sdk2_handcmd.fake_sdk import _Bus, _install_fake_sdk

CFG = "unitree_g1_dex3"


# ----------------------------------------------------------------- fake SDK


LEFT_Q = [0.35, 0.39, 0.65, -1.51, -1.63, -1.12, -1.43]   # read off the real left hand
RIGHT_Q = [0.50, -0.19, -1.49, 1.38, 1.68, 1.35, 1.71]


def _hands(monkeypatch, read_only=False):
    bus = _Bus()
    _install_fake_sdk(monkeypatch, bus, {"rt/dex3/left/state": LEFT_Q, "rt/dex3/right/state": RIGHT_Q})
    emb = load_embodiment(CFG)
    return emb, bus, build_end_effectors(emb, "eth0", read_only=read_only)


# --------------------------------------------------------------- the config


def test_config_lists_43_joints_with_hand_ownership():
    emb = load_embodiment(CFG)
    assert emb.num_dofs == 43
    names = emb.joint_names
    assert names[29] == "left_hand_thumb_0" and names[42] == "right_hand_index_1"
    assert end_effector_joint_names(emb) == names[29:]
    body = body_embodiment(emb)
    assert body.joint_names == names[:29]
    assert [ee.name for ee in body.end_effectors] == ["left_hand", "right_hand"]
    assert all(not ee.joints and ee.driver is None for ee in body.end_effectors)
    lt = emb.joints[names.index("left_hand_middle_0")]
    assert lt.position_limit == (-1.5708, 0.0) and lt.torque_limit == 1.4


def test_embodiment_rejects_bad_hand_joints():
    from hirobot import Embodiment

    with pytest.raises(ValueError, match="unknown joint"):
        Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[{"name": "h", "joints": ["b"]}])
    with pytest.raises(ValueError, match="listed by both"):
        Embodiment(name="x", joints=[{"name": "a"}], end_effectors=[
            {"name": "h", "joints": ["a"]}, {"name": "g", "joints": ["a"]}])
    with pytest.raises(ValueError, match="duplicate"):
        Embodiment(name="x", joints=[{"name": "a"}, {"name": "a"}])


def test_sim_driver_gets_all_43_joints_and_no_hand_driver():
    robot = make_robot(CFG)
    assert isinstance(robot, ToyRobot)
    assert robot.embodiment.num_dofs == 43


def test_real_driver_gets_body_joints_and_two_hands():
    robot = make_robot(CFG, overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth0"}}})
    assert isinstance(robot, RobotAssembly)
    assert robot.embodiment.num_dofs == 43
    assert robot.body.embodiment.num_dofs == 29
    assert set(robot.end_effectors) == {"left_hand", "right_hand"}
    left = robot.end_effectors["left_hand"]
    assert left.cmd_topic == "rt/dex3/left/cmd" and left._network_interface == "eth0"
    assert robot.safety.num_dofs == 43


def test_read_only_body_makes_hands_read_only():
    robot = make_robot(CFG, overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth0",
                                             "read_only": True}}})
    assert all(h.read_only for h in robot.end_effectors.values())


def test_registry():
    assert "unitree_handcmd" in available_end_effectors()
    assert get_end_effector("unitree_handcmd") is UnitreeDex3Hand
    with pytest.raises(KeyError, match="Unknown end-effector driver"):
        get_end_effector("nope")


# ------------------------------------------------------------- the driver


# ------------------------------------------------------------- the assembly


class _Body(ToyRobot):
    """Toy body that records what reaches it."""

    def __init__(self, emb):
        super().__init__(emb)
        self.sent = []
        self.damped = 0

    def _send_action(self, action):
        self.sent.append(action)
        super()._send_action(action)

    def damp(self):
        self.damped += 1


def _assembly(monkeypatch):
    emb, bus, hands = _hands(monkeypatch)
    body = _Body(body_embodiment(emb))
    return RobotAssembly(body, end_effectors=hands, embodiment=emb), body, bus


def test_observation_joins_body_and_hands(monkeypatch):
    robot, body, _ = _assembly(monkeypatch)
    robot.connect()
    obs = robot.get_observation()
    assert obs.joint_pos.shape == (43,)
    assert np.allclose(obs.joint_pos[29:36], LEFT_Q) and np.allclose(obs.joint_pos[36:], RIGHT_Q)
    assert np.allclose(obs.joint_pos[:29], body.get_observation().joint_pos)
    assert obs.extra["left_hand_pressure"].shape == (9, 12)


def test_action_is_split_by_joint(monkeypatch):
    robot, body, bus = _assembly(monkeypatch)
    robot.connect()
    target = np.zeros(43)
    target[18] = 0.4                          # left_elbow
    target[29:36] = [0.1, 0.2, 0.3, -0.4, -0.5, -0.6, -0.7]
    target[36:] = [-0.1, -0.2, -0.3, 0.4, 0.5, 0.6, 0.7]
    kp = np.arange(43, dtype=float)
    robot.send_action(Action(mode=ControlMode.PD, joint_target=target, kp=kp))
    sent = body.sent[-1]
    assert sent.joint_target.shape == (29,) and sent.joint_target[18] == 0.4
    assert np.array_equal(sent.kp, np.arange(29, dtype=float))
    left = bus.sent["rt/dex3/left/cmd"][-1]
    assert np.allclose([m[1] for m in left], target[29:36])
    assert [m[2] for m in left] == list(range(29, 36))
    right = bus.sent["rt/dex3/right/cmd"][-1]
    assert np.allclose([m[1] for m in right], target[36:])
    assert [m[3] for m in right] == [0.2] * 7  # kd not given: hand default


def test_safety_gate_sees_all_43_joints(monkeypatch):
    from hirobot import SafetyError, SafetyGate

    robot, body, bus = _assembly(monkeypatch)
    robot._set_safety(SafetyGate.from_embodiment(robot.embodiment))
    robot.connect()
    with pytest.raises(SafetyError, match="43"):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=np.zeros(29)))
    assert not body.sent and not bus.sent["rt/dex3/left/cmd"]


def test_hands_refuse_torque_mode(monkeypatch):
    robot, body, bus = _assembly(monkeypatch)
    robot.connect()
    with pytest.raises(ValueError, match="position targets"):
        robot.send_action(Action(mode=ControlMode.TORQUE, joint_target=np.zeros(43)))
    assert not body.sent and not bus.sent["rt/dex3/left/cmd"]


def test_disconnect_damps_both_hands(monkeypatch):
    robot, body, bus = _assembly(monkeypatch)
    robot.connect()
    robot.disconnect()
    for topic in ("rt/dex3/left/cmd", "rt/dex3/right/cmd"):
        assert all(m[0] & 0x80 and m[2] == 0.0 for m in bus.sent[topic][-1])


# ---------------------------------------------------------------- MuJoCo with hands

MJ_CFG = "unitree_g1_dex3_mujoco"
_MODEL = "~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml"


def _mujoco_or_skip():
    import os

    pytest.importorskip("mujoco")
    if not os.path.isfile(os.path.expanduser(_MODEL)):
        pytest.skip(f"G1 + Dex3 MJCF not found at {_MODEL}")


def test_mujoco_fingers_track_open_and_close():
    _mujoco_or_skip()
    robot = make_robot(MJ_CFG, overrides={"body": {"driver": {"realtime": False, "viewer": False}}})
    robot.connect()
    robot.reset()
    emb = robot.embodiment
    lo = np.array([j.position_limit[0] for j in emb.joints[29:]])
    hi = np.array([j.position_limit[1] for j in emb.joints[29:]])
    # Let the arms finish sagging under gravity before taking the reference pose.
    hold = robot.get_observation().joint_pos.copy()
    for _ in range(2 * emb.control_hz):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=hold))
    start = robot.get_observation().joint_pos.copy()
    start[:29] = hold[:29]  # keep commanding the same body target
    settled = robot.get_observation().joint_pos[:29].copy()
    for goal in (lo + 0.3 * (hi - lo), lo + 0.7 * (hi - lo)):
        target = start.copy()
        target[29:] = goal
        for _ in range(2 * emb.control_hz):  # 2 s
            robot.send_action(Action(mode=ControlMode.PD, joint_target=target))
        q = robot.get_observation().joint_pos
        assert np.all(np.isfinite(q))
        assert np.abs(q[29:] - goal).max() < 0.05
        assert np.abs(q[:29] - settled).max() < 0.02  # moving the fingers leaves the body still
    robot.disconnect()
