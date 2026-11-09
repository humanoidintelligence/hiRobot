"""``fourier_grx`` passes the body contract against a fake ``fourier_grx_client``."""

import numpy as np

from hirobot import Action, ControlMode, get_body, load_embodiment
from tests.drivers.body.fourier.grx import fake_sdk
from tests.drivers.contract import check_robot


def _sent(client_cls):
    return sum(1 for c in client_cls.instances for call in c.calls if call[0] == "control_joints")


def test_contract(monkeypatch):
    emb = load_embodiment("fourier_gr1t2")
    client_cls = fake_sdk.install(monkeypatch, num_joints=emb.num_dofs)
    check_robot("fourier_grx", lambda: get_body("fourier_grx")(emb), lambda: _sent(client_cls))


def test_modes_gains_and_power(monkeypatch):
    emb = load_embodiment("fourier_gr1t1")
    client_cls = fake_sdk.install(monkeypatch, num_joints=emb.num_dofs)
    robot = get_body("fourier_grx")(emb, server_ip="10.0.0.2")
    robot.connect()
    client = client_cls.instances[-1]
    assert client.kwargs["server_ip"] == "10.0.0.2" and client.calls[0] == ("set_enable", True)
    obs = robot.get_observation()
    assert np.allclose(obs.base_angular_velocity, [np.pi / 2, 0, 0])  # deg/s -> rad/s
    n = emb.num_dofs
    robot.send_action(Action(mode=ControlMode.PD, joint_target=np.zeros(n),
                             kp=np.full(n, 50.0), kd=np.full(n, 1.0)))
    robot.send_action(Action(mode=ControlMode.TORQUE, joint_target=np.ones(n)))
    kinds = [c[1] for c in client.calls if c[0] == "control_joints"]
    assert kinds == ["position", "effort"]
    assert any(c[0] == "set_gains" and np.all(c[1] == 50.0) for c in client.calls)
    robot.disconnect()
    assert client.calls[-2:] == [("set_enable", False), ("close",)]
