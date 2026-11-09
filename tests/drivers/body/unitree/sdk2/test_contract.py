"""``unitree_sdk2`` passes the body contract against the fake SDK (hg G1 and go H1)."""

import pytest

from hirobot import get_body, load_embodiment
from tests.drivers.body.unitree.sdk2 import fake_sdk
from tests.drivers.contract import check_robot


@pytest.mark.parametrize("config,family,slots",
                         [("unitree_g1", "hg", 35), ("unitree_h1", "go", 20)])
def test_contract(monkeypatch, config, family, slots):
    emb = load_embodiment(config)
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * slots, slots=slots,
                                                           quat_wxyz=(0.9239, 0.0, 0.0, 0.3827)))
    check_robot("unitree_sdk2",
                lambda: get_body("unitree_sdk2")(emb, msg_family=family, ramp_seconds=0.0),
                lambda: len(bus.commands()))


def test_read_only_never_creates_a_publisher(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.0] * 35))
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"), read_only=True)
    robot.connect()
    assert robot.get_observation().joint_pos.shape == (29,)
    with pytest.raises(RuntimeError, match="read_only"):
        robot.reset()
    robot.disconnect()
    assert bus.publishers == []


def test_disconnect_leaves_the_robot_damped(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.0] * 35))
    robot = get_body("unitree_sdk2")(load_embodiment("unitree_g1"), damping_kd=8.0)
    robot.connect()
    robot.disconnect()
    last = bus.commands()[-1][:29]
    assert all(kp == 0.0 and kd == 8.0 for _, _, _, _, _, kp, kd in last)


def test_connect_times_out_without_lowstate(monkeypatch):
    import time

    fake_sdk.install(monkeypatch, None)
    monkeypatch.setattr(time, "sleep", lambda s: None)
    with pytest.raises(TimeoutError, match="LowState"):
        get_body("unitree_sdk2")(load_embodiment("unitree_g1")).connect()
