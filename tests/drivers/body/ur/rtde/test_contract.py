"""``ur_rtde`` passes the body contract against a fake ``ur_rtde``."""

import numpy as np

from hirobot import Action, ControlMode, get_body, load_embodiment
from hirobot.geometry.transforms import pose_from_axis_angle
from tests.drivers.body.ur.rtde import fake_sdk
from tests.drivers.contract import check_robot

MOTION = ("servoJ", "speedJ", "servoL")


def test_contract(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    check_robot("ur_rtde", lambda: get_body("ur_rtde")(load_embodiment("ur_ur5e"), ip="10.0.0.5"),
                lambda: sum(1 for c in arm.calls if c[0] in MOTION))


def test_modes_and_shutdown(monkeypatch):
    arm = fake_sdk.install(monkeypatch)
    robot = get_body("ur_rtde")(load_embodiment("ur_ur5e"), home_q=[0.0] * 6)
    robot.connect()
    robot.reset()
    assert ("moveJ", [0.0] * 6, 0.5, 0.5, True) in arm.calls
    obs = robot.get_observation()
    assert obs.ee_poses["tcp"].shape == (7,)  # 7-vector, not RTDE's axis-angle
    assert np.allclose(obs.ee_poses["tcp"], pose_from_axis_angle(arm.tcp))
    robot.send_action(Action(mode=ControlMode.VELOCITY, joint_target=np.zeros(6)))
    robot.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"tcp": obs.ee_poses["tcp"]}))
    assert [c[0] for c in arm.calls if c[0] in MOTION] == ["speedJ", "servoL"]
    servo = next(c for c in arm.calls if c[0] == "servoL")
    assert np.allclose(pose_from_axis_angle(servo[1]), pose_from_axis_angle(arm.tcp))
    robot.disconnect()
    names = [c[0] for c in arm.calls]
    assert names[-4:] == ["servoStop", "stopScript", "disconnect", "receive.disconnect"]
