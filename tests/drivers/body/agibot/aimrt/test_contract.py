"""``agibot_aimrt`` passes the body contract against a fake ROS 2."""

import numpy as np
import pytest

from hirobot import Action, ControlMode, get_body, load_embodiment
from tests.drivers.body.agibot.aimrt import fake_sdk
from tests.drivers.contract import check_robot


def _robot(monkeypatch):
    emb = load_embodiment("agibot_x1")
    ros = fake_sdk.install(monkeypatch)
    ros.joint_state(np.zeros(emb.num_dofs), np.zeros(emb.num_dofs), np.zeros(emb.num_dofs))
    ros.imu((0.0, 0.0, 0.3827, 0.9239))
    return emb, ros


def test_contract(monkeypatch):
    emb, ros = _robot(monkeypatch)
    check_robot("agibot_aimrt", lambda: get_body("agibot_aimrt")(emb), lambda: len(ros.published))
    assert ros.destroyed and not ros.ok


def test_command_is_by_name_with_gains(monkeypatch):
    emb, ros = _robot(monkeypatch)
    robot = get_body("agibot_aimrt")(emb, default_kp=80.0)
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.base_orientation, [0.0, 0.0, 0.3827, 0.9239])
    robot.send_action(Action(mode=ControlMode.PD, joint_target=np.full(emb.num_dofs, 0.1)))
    topic, cmd = ros.published[-1]
    assert topic == "/joint_cmd" and cmd.name == emb.joint_names
    assert cmd.stiffness == [80.0] * emb.num_dofs and cmd.position == [0.1] * emb.num_dofs
    robot.disconnect()


def test_connect_times_out_without_joint_states(monkeypatch):
    emb = load_embodiment("agibot_x1")
    fake_sdk.install(monkeypatch)
    with pytest.raises(TimeoutError, match="/joint_states"):
        get_body("agibot_aimrt")(emb, spin_timeout=0.04).connect()
