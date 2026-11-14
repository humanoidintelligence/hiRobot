"""``trossen_dynamixel`` passes the body contract against a fake dynamixel_sdk."""

import numpy as np
import pytest

from hirobot import Action, ControlMode, get_body, load_embodiment
from tests.drivers.body.trossen.dynamixel import fake_sdk
from tests.drivers.contract import check_robot

CENTER = 2048
TWO_PI_OVER_REV = 2.0 * np.pi / 4096


@pytest.fixture(autouse=True)
def _sdk_found(monkeypatch):
    # The fake has no package metadata; skip the manifest version check.
    monkeypatch.setattr("hirobot.drivers.body.trossen.dynamixel.driver.require_sdk",
                        lambda *a, **k: {})


def _robot(**kw):
    return get_body("trossen_dynamixel")(load_embodiment("body/trossen/wx250s"), **kw)


def test_contract(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    check_robot("trossen_dynamixel", _robot, lambda: len(fake_sdk.goal_writes(bus)))
    assert bus.ports[0] == "/dev/ttyDXL"


def test_connect_sends_nothing(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot(port="/dev/ttyUSB1")
    robot.connect()
    robot.get_observation()
    robot.get_status()
    robot.disconnect()
    assert bus.writes == []                    # no torque, no goal while only reading
    assert bus.torque == {sid: 0 for sid in fake_sdk.START_POS}
    assert bus.ports == ["/dev/ttyUSB1"] and bus.closed == 1


def test_connect_without_servos_raises(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.present = False
    robot = _robot(feedback_timeout_s=0.05)
    with pytest.raises(ConnectionError, match="no answer from servo"):
        robot.connect()
    assert bus.closed == 1


def test_observation_units(monkeypatch):
    fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    obs = robot.get_observation()
    expected = [(fake_sdk.START_POS[i] - CENTER) * TWO_PI_OVER_REV for i in (1, 2, 4, 6, 7, 8)]
    assert np.allclose(obs.joint_pos, expected)
    assert obs.joint_vel is not None and obs.joint_vel.shape == (6,)
    # gripper: 2936 ticks on a 1780(closed)..2950(open) map -> near fully open.
    assert obs.gripper["ee"] == pytest.approx((2936 - 1780) / (2950 - 1780))
    assert obs.stamps["ee"] == obs.stamps["body"]
    status = robot.get_status()
    assert status.joint_temperature.tolist() == [34.0] * 6
    assert status.extra["voltage_v"] == pytest.approx(12.1)
    assert status.extra["enabled"] == [False] * 6
    robot.disconnect()


def test_gripper_opening_maps_linearly(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.pos[9] = 2200                          # between closed(1780) and open(2950)
    robot = _robot()
    robot.connect()
    obs = robot.get_observation()
    assert obs.gripper["ee"] == pytest.approx((2200 - 1780) / (2950 - 1780))
    robot.disconnect()


def test_joint_move_and_gripper(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    q = np.array([0.0, 0.1, -0.2, 0.0, 0.3, 0.0])
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q, gripper={"ee": 1.0}))
    goals = dict(fake_sdk.goal_writes(bus))     # last write per servo wins
    for sid, rad in zip((1, 2, 4, 6, 7, 8), q):
        assert goals[sid] == int(round(rad / TWO_PI_OVER_REV)) + CENTER
    assert goals[9] == 2950                     # opening 1.0 -> open tick
    assert bus.torque == {sid: 1 for sid in fake_sdk.START_POS}
    robot.disconnect()


def test_enable_holds_position_then_torques_on(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=None, gripper={"ee": 0.5}))
    # Every arm servo was told its own measured position before torque came on.
    goals = fake_sdk.goal_writes(bus)
    holds = {sid: tick for sid, tick in goals}
    for sid in (1, 2, 4, 6, 7, 8):
        assert holds[sid] == fake_sdk.START_POS[sid]
    robot.disconnect()


def test_gripper_out_of_range_sends_nothing(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    with pytest.raises(ValueError, match="outside"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6),
                                 gripper={"ee": 1.2}))
    assert bus.writes == []
    robot.disconnect()


def test_gripper_off_leaves_it_alone(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot(gripper=False)
    robot.connect()
    assert robot.get_observation().gripper == {}
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(6), gripper={"ee": 0.0}))
    assert 9 not in dict(fake_sdk.goal_writes(bus))      # gripper servo untouched
    robot.disconnect()


def test_estop_holds_without_enabling(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    robot.estop()                              # torque never enabled -> no goal written
    assert fake_sdk.goal_writes(bus) == []
    assert bus.torque == {sid: 0 for sid in fake_sdk.START_POS}
    robot.disconnect()


def test_estop_after_move_holds_measured(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    robot = _robot()
    robot.connect()
    robot.send_action(Action(mode=ControlMode.POSITION,
                             joint_target=np.zeros(6), gripper={"ee": 0.0}))
    robot.estop()
    held = dict(fake_sdk.goal_writes(bus))     # last goal per servo = measured position
    for sid in (1, 2, 4, 6, 7, 8):
        assert held[sid] == CENTER             # the zeros we just commanded, now measured
    robot.disconnect()


def test_bad_arguments():
    emb = load_embodiment("body/trossen/wx250s")
    cls = get_body("trossen_dynamixel")
    with pytest.raises(ValueError, match="differ"):
        cls(emb, gripper_close_tick=2000, gripper_open_tick=2000)
    with pytest.raises(ValueError, match="6 joints"):
        cls(load_embodiment("body/franka/fr3_droid"))
