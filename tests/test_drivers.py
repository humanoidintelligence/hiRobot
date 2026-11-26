"""Smoke tests for driver registration and construction.

These do not touch hardware: they verify every config builds, every documented
driver is registered, and the real drivers construct without their vendor SDK
and fail with a clear error on connect() (lazy import). Hardware behavior is
covered manually during setup.
"""

from __future__ import annotations

import importlib.util
import os

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
from hirobot.loading.config import available_configs
from hirobot.loading.tree import DeviceSpec, load_robot

CONFIGS = available_configs("robots")
# Configs whose driver runs without an SDK install (toy bodies). A `sim:` section
# puts them into a shared MuJoCo world, so those need mujoco and skip without it.
# They also need every body's MJCF (`hirobot assets fetch`) and skip without it.
_NO_MUJOCO = pytest.mark.skipif(importlib.util.find_spec("mujoco") is None, reason="mujoco not installed")


def _bodies(cfg):
    return [d for _, d in load_robot(cfg).root.walk()
            if isinstance(d, DeviceSpec) and d.kind == "body"]


def _sim_param(cfg):
    missing = [os.path.expanduser(m) for d in _bodies(cfg)
               if (m := (d.fields.get("model") or {}).get("mjcf"))
               and not os.path.isfile(os.path.expanduser(m))]
    marks = [_NO_MUJOCO]
    if missing:
        marks.append(pytest.mark.skip(reason=f"MJCF not found: {missing[0]}"))
    return pytest.param(cfg, marks=marks)


SIM_CONFIGS = [_sim_param(c) if load_robot(c).sim is not None else c
               for c in CONFIGS
               if all((d.driver or {}).get("type", "toy") == "toy" for d in _bodies(c))]
REAL_DRIVERS = {
    "unitree_sdk2": {"network_interface": "eth0"},
    "fourier_grx": {"server_ip": "localhost"},
    "ur_rtde": {"ip": "127.0.0.1"},
    "ros2": {"state": [{"type": "joint_state", "topic": "/joint_states"}]},
}


def test_expected_drivers_registered():
    for name in ["toy", "mujoco", "unitree_sdk2", "fourier_grx", "ur_rtde", "franka_droid", "ros2"]:
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
        grippers = [e.name for e in robot.embodiment.end_effectors if e.has_gripper][:1]
        chunk = ActionChunk(
            mode=ControlMode.PD,
            joint_targets=np.zeros((5, n)),
            kp=np.full(n, 20.0),
            kd=np.full(n, 1.0),
            gripper={g: np.linspace(0, 1, 5) for g in grippers},
        )
        for t in range(12):
            robot.get_observation()
            robot.send_action(chunk.step(t % 3))
        assert robot.get_observation().joint_pos is not None
    finally:
        robot.disconnect()


