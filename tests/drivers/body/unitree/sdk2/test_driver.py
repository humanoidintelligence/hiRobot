"""``unitree_sdk2``: gains, robot overrides, and no MuJoCo import without sim."""

from __future__ import annotations

import numpy as np
import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import (
    get_body,
    load_embodiment,
    make_robot,
)


def test_unitree_per_joint_gains():
    emb = load_embodiment("unitree_g1")
    cls = get_body("unitree_sdk2")
    kp = list(range(emb.num_dofs))
    robot = cls(emb, default_kp=kp, default_kd=2.0)
    assert np.array_equal(robot._kp, np.arange(emb.num_dofs, dtype=float))
    assert np.all(robot._kd == 2.0)
    with pytest.raises(ValueError, match="default_kp has 3 values"):
        cls(emb, default_kp=[1.0, 2.0, 3.0])


def test_make_robot_overrides_body_section():
    robot = make_robot(
        "unitree_g1",
        overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth9", "msg_family": "hg"}}},
    )
    # The G1 config declares a head camera, so the body comes wrapped in a RobotAssembly.
    assert type(robot).__name__ == "RobotAssembly"
    assert type(robot.body).__name__ == "UnitreeRobot"
    assert robot.body._network_interface == "eth9"


def test_real_robot_without_sim_does_not_import_mujoco():
    """sim off (the default) must work on a host without MuJoCo installed."""
    import subprocess
    import sys

    code = (
        "import sys; from hirobot import make_robot; "
        "r = make_robot('unitree_g1', "
        "overrides={'body': {'driver': {'type': 'unitree_sdk2', 'network_interface': 'eth9'}}}); "
        "assert type(r.body).__name__ == 'UnitreeRobot'; "
        "assert 'mujoco' not in sys.modules, 'mujoco imported'"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
