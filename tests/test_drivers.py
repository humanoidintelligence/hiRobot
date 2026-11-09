"""Smoke tests for driver registration and construction.

These do not touch hardware: they verify every config builds, every documented
driver is registered, and the real drivers construct without their vendor SDK
and fail with a clear error on connect() (lazy import). Hardware behavior is
covered manually during setup.
"""

from __future__ import annotations

import numpy as np
import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import (
    ActionChunk,
    ControlMode,
    available_bodies,
    get_body,
    load_embodiment,
    make_robot,
)
from hirobot.config import available_configs, load_config

CONFIGS = available_configs("robots")
# Configs whose driver runs without an SDK/simulator install (the toy sim).
SIM_CONFIGS = [c for c in CONFIGS if load_config(c)["body"]["driver"] == "toy"]
REAL_DRIVERS = {
    "unitree_sdk2": {"network_interface": "eth0"},
    "fourier_grx": {"server_ip": "localhost"},
    "ur_rtde": {"ip": "127.0.0.1"},
    "franka_libfranka": {"ip": "172.16.0.2"},
    "agibot_aimrt": {},
}


def test_expected_drivers_registered():
    for name in ["toy", "mujoco", "unitree_sdk2", "fourier_grx", "ur_rtde", "franka_libfranka", "agibot_aimrt"]:
        assert name in available_bodies()


@pytest.mark.parametrize("cfg", CONFIGS)
def test_every_config_builds(cfg):
    robot = make_robot(cfg)
    assert robot.embodiment.num_dofs > 0
    assert robot.embodiment.joint_names


@pytest.mark.parametrize("name,kwargs", list(REAL_DRIVERS.items()))
def test_real_driver_constructs_and_fails_cleanly(name, kwargs):
    emb = load_embodiment("unitree_g1")
    driver = get_body(name)(emb, **kwargs)
    assert driver.embodiment is emb
    # SDK absent in CI: connect() must raise a clear ImportError/Timeout, not
    # crash at import time.
    with pytest.raises((ImportError, TimeoutError, RuntimeError)):
        driver.connect()


@pytest.mark.parametrize("cfg", SIM_CONFIGS)
def test_sim_steps_pd_actions(cfg):
    robot = make_robot(cfg)
    robot.connect()
    try:
        robot.reset()
        n = robot.embodiment.num_dofs
        chunk = ActionChunk(
            mode=ControlMode.PD,
            joint_targets=np.zeros((5, n)),
            kp=np.full(n, 20.0),
            kd=np.full(n, 1.0),
            gripper={"g": np.linspace(0, 1, 5)},
        )
        for t in range(12):
            robot.get_observation()
            robot.send_action(chunk.step(t % 3))
        assert robot.get_observation().joint_pos is not None
    finally:
        robot.disconnect()


