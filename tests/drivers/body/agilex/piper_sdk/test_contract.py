"""``agilex_piper_sdk`` passes the body contract against a fake piper_sdk."""

import math

import numpy as np
import pytest

from hirobot import Action, ControlMode, get_body, load_embodiment
from hirobot.geometry.transforms import quat_to_matrix
from tests.drivers.body.agilex.piper_sdk import fake_sdk
from tests.drivers.contract import check_robot


@pytest.fixture(autouse=True)
def _sdk_found(monkeypatch):
    # The fake has no package metadata; skip the manifest version check.
    monkeypatch.setattr("hirobot.drivers.body.agilex.piper_sdk.driver.require_sdk",
                        lambda *a, **k: {})


def _robot(**kw):
    return get_body("agilex_piper_sdk")(load_embodiment("body/agilex/piper"), **kw)


def test_contract(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    check_robot("agilex_piper_sdk", _robot, lambda: len(arm.commands()))
    assert arm.can_names[0] == "can0"


def test_connect_sends_nothing(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = _robot(can="can1")
    robot.connect()
    robot.get_observation()
    robot.get_status()
    robot.disconnect()
    assert arm.connect_kwargs == [{"piper_init": False}]
    assert arm.commands() == []
    assert arm.can_names == ["can1"] and arm.disconnected == 1


def test_connect_without_feedback_raises(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    arm.feedback = False
    robot = _robot(feedback_timeout_s=0.05)
    with pytest.raises(ConnectionError, match="no PiPER joint feedback on can0"):
        robot.connect()
    assert arm.disconnected == 1


def test_observation_units(monkeypatch):
    fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.joint_pos, np.radians([-0.116, 0.12, 0, 0, 0, 0]))
    pose = obs.ee_poses["ee"]
    assert np.allclose(pose[:3], [0.056316, -0.000114, 0.213148])
    # RX 0, RY 85.12 deg, RZ -0.116 deg, R = Rz Ry Rx.
    m = quat_to_matrix(pose[3:])
    ry, rz = math.radians(85.12), math.radians(-0.116)
    assert np.isclose(m[2, 0], -math.sin(ry)) and np.isclose(math.atan2(m[1, 0], m[0, 0]), rz)
    assert obs.gripper["ee"] == pytest.approx(0.0602 / 0.07)
    assert obs.stamps["ee"] == obs.stamps["body"]
    status = robot.get_status()
    assert status.joint_temperature.tolist() == [33.0] * 6
    assert status.extra["voltage_v"] == pytest.approx(23.0)
    assert status.extra["enabled"] == [True] * 6
    robot.disconnect()


def test_joint_move_and_gripper(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = _robot(speed_percent=10, gripper_effort_n=1.5)
    robot.connect()
    q = np.radians([10.0, 20.0, -30.0, 0.5, -1.0, 2.0])
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q, gripper={"ee": 0.5}))
    assert arm.commands() == [
        ("MotionCtrl_2", (0x01, 0x01, 10, 0x00)),
        ("JointCtrl", (10000, 20000, -30000, 500, -1000, 2000)),
        ("GripperCtrl", (35000, 1500, 0x01, 0x00)),
    ]
    robot.disconnect()


def test_enables_disabled_arm_before_moving(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    arm.enabled = [False] * 6
    robot = _robot()
    robot.connect()
    assert arm.commands() == []
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6)))
    assert [c[0] for c in arm.commands()] == ["EnableArm", "MotionCtrl_2", "JointCtrl"]
    robot.disconnect()


def test_gripper_out_of_range_sends_nothing(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    with pytest.raises(ValueError, match="outside"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6),
                                 gripper={"ee": 1.2}))
    assert arm.commands() == []
    robot.disconnect()


def test_gripper_off_leaves_it_alone(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = _robot(gripper=False)
    robot.connect()
    assert robot.get_observation().gripper == {}
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6), gripper={"ee": 0.0}))
    assert [c[0] for c in arm.commands()] == ["MotionCtrl_2", "JointCtrl"]
    robot.disconnect()


def test_estop_holds_measured_position(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    robot.estop()
    assert arm.commands()[-1] == ("JointCtrl", tuple(fake_sdk.Arm().mdeg))
    assert "EmergencyStop" not in [c[0] for c in arm.commands()]
    robot.disconnect()


def test_estop_does_not_enable(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    arm.enabled = [False] * 6
    robot = _robot()
    robot.connect()
    robot.estop()
    assert "EnableArm" not in [c[0] for c in arm.commands()]
    assert arm.enabled == [False] * 6
    robot.disconnect()


def test_bad_arguments():
    emb = load_embodiment("body/agilex/piper")
    cls = get_body("agilex_piper_sdk")
    with pytest.raises(ValueError, match="speed_percent"):
        cls(emb, speed_percent=0)
    with pytest.raises(ValueError, match="gripper_effort_n"):
        cls(emb, gripper_effort_n=6)
    with pytest.raises(ValueError, match="6 joints"):
        cls(load_embodiment("body/franka/fr3_droid"))
