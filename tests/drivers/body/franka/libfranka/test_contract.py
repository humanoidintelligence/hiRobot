"""``franka_libfranka`` passes the body contract against a fake ``pylibfranka``."""

import numpy as np
import pytest

from hirobot import Action, ControlMode, get_body, load_embodiment
from tests.drivers.body.franka.libfranka import fake_sdk
from tests.drivers.contract import check_robot


def test_contract(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    check_robot("franka_libfranka",
                lambda: get_body("franka_libfranka")(load_embodiment("franka_fr3"),
                                                       realtime=False),
                lambda: len(arm.targets))


def test_needs_reset_before_commands_and_stops_the_handler(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = get_body("franka_libfranka")(load_embodiment("franka_fr3"))
    robot.connect()
    assert arm.calls[0] == ("Robot", "172.16.0.2", "enforce")
    with pytest.raises(RuntimeError, match="reset"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array(arm.q)))
    robot.reset()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array(arm.q)))
    assert arm.targets == [arm.q]
    robot.disconnect()
    assert ("handler.stop",) in arm.calls


def test_ee_pose_is_a_7_vector(monkeypatch):
    fake_sdk.install(monkeypatch)
    robot = get_body("franka_libfranka")(load_embodiment("franka_fr3"))
    robot.connect()
    pose = robot.get_observation().ee_poses["ee"]
    # Column-major O_T_EE read the right way round: translation is the last column.
    assert pose.shape == (7,)
    assert np.allclose(pose[:3], [0.3, 0.0, 0.5])
    assert np.allclose(np.abs(pose[3:]), [1.0, 0.0, 0.0, 0.0])  # 180 deg about x
