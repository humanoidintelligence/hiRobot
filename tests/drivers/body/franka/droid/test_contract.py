"""``franka_droid`` passes the body contract against a fake DROID server."""

import threading

import numpy as np
import pytest

from hirobot import Action, ControlMode, get_body, load_embodiment
from hirobot.drivers.body.franka.droid.driver import euler_xyz_to_quat, quat_to_euler_xyz
from tests.drivers.body.franka.droid import fake_sdk
from tests.drivers.contract import check_robot


@pytest.fixture(autouse=True)
def _sdk_found(monkeypatch):
    # The fake has no package metadata; skip the manifest version check.
    monkeypatch.setattr("hirobot.drivers.body.franka.droid.driver.require_sdk",
                        lambda *a, **k: {})


def _robot(**kw):
    return get_body("franka_droid")(load_embodiment("body/franka/fr3_droid"), **kw)


def test_contract(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    check_robot("franka_droid", _robot, lambda: len(server.commands()))
    assert server.connected[0] == "tcp://172.16.0.2:4242"


def test_observation(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.joint_pos, server.q)
    pose = obs.ee_poses["ee"]
    assert np.allclose(pose[:3], [0.36, 0.0, 0.37])
    assert np.allclose(np.abs(pose[3:]), [1.0, 0.0, 0.0, 0.0])  # 180 deg about x
    assert obs.gripper["ee"] == pytest.approx(0.75)  # DROID 0.25 closed -> 0.75 open
    assert obs.stamps["ee"] == obs.stamps["body"]
    robot.disconnect()
    assert server.closed == 1


def test_commands_use_upstream_calls(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    q = np.array(server.q) + 0.01
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q, gripper={"ee": 1.0}))
    assert server.commands()[-2][:2] == ("update_joints", (q.tolist(), False, False))
    assert server.commands()[-1][:2] == ("update_gripper", (0.0, False, False))  # 1 open -> 0 closed
    target = np.array([0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])
    robot.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"ee": target}))
    method, (pose, velocity, blocking), _ = server.commands()[-1]
    assert method == "update_pose" and velocity is False and blocking is False
    assert np.allclose(pose[:3], [0.4, 0.0, 0.3])
    assert np.allclose(euler_xyz_to_quat(pose[3:]), [1.0, 0.0, 0.0, 0.0]) or \
        np.allclose(euler_xyz_to_quat(pose[3:]), [-1.0, 0.0, 0.0, 0.0])
    robot.disconnect()


def test_gripper_off_leaves_it_alone(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    robot = _robot(gripper=False)
    robot.connect()
    assert robot.get_observation().gripper == {}
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array(server.q),
                             gripper={"ee": 0.0}))
    assert [c[0] for c in server.commands()] == ["update_joints"]
    robot.disconnect()


def test_reset_moves_home_blocking(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    home = [0.0, -0.5, 0.0, -2.4, 0.0, 1.9, 0.8]
    robot = _robot(home_q=home)
    robot.connect()
    robot.reset()
    assert server.commands()[-1][:2] == ("update_joints", (home, False, True))
    robot.disconnect()


def test_estop_holds_measured_joints_from_another_thread(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    robot.get_observation()
    t = threading.Thread(target=robot.estop, name="estop-thread")
    t.start()
    t.join(5)
    method, args, thread = server.commands()[-1]
    assert (method, args) == ("update_joints", (server.q, False, False))
    assert thread == "estop-thread"
    assert robot.fault is not None
    robot.disconnect()
    assert server.closed == 2  # one client per thread, all closed


def test_connect_fails_clearly_when_server_does_not_answer(monkeypatch):
    server = fake_sdk.install(monkeypatch)
    server.fail = {"get_robot_state": RuntimeError("Impedance controller fault")}
    with pytest.raises(ConnectionError, match="did not answer get_robot_state"):
        _robot().connect()


def test_home_q_length_checked():
    with pytest.raises(ValueError, match="home_q has 6 values"):
        _robot(home_q=[0.0] * 6)


# scipy.spatial.transform.Rotation.from_euler("xyz", rpy).as_quat() (DROID's convention),
# computed with scipy 1.x; kept as numbers so the test needs no scipy.
@pytest.mark.parametrize("rpy, ref", [
    ([0.1, -0.4, 2.0], [0.1934311339, -0.0659896482, 0.8290317701, 0.520515213]),
    ([3.12, 0.046, 0.87], [0.9064727161, 0.4214995557, -0.0163065638, 0.0194791055]),
    ([-2.0, 1.2, -0.3], [-0.6411073002, 0.4054361186, 0.4031560011, 0.5119259928]),
])
def test_euler_matches_droid_convention(rpy, ref):
    q = euler_xyz_to_quat(rpy)
    assert np.allclose(q, ref, atol=1e-8) or np.allclose(q, -np.asarray(ref), atol=1e-8)
    assert np.allclose(quat_to_euler_xyz(q), rpy)
