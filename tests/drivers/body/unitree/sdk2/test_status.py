"""``unitree_sdk2`` get_status / IMU mapping (hg and go families)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import BatteryState, get_body, load_embodiment

G1 = "unitree_g1"


H1 = "unitree_h1"


def _hg_motor(temp, fault=0, vol=48.5):
    return SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0, temperature=[temp, temp - 1],
                           vol=vol, motorstate=fault)


# Values read off a real G1 on 2026-11-05 (rt/lf/bmsstate, rt/lf/mainboardstate).
_G1_BMS = SimpleNamespace(
    soc=49, soh=93, cycle=26, current=-2239, bmsvoltage=[48378, 48400, 0],
    temperature=[26, 24, 24, 27] + [0] * 8, cell_vol=[3725, 3726, 3727] + [0] * 37,
)


_G1_BOARD = SimpleNamespace(fan_state=[0] * 6, temperature=[42, 0, 0, 0, 0, 0],
                            value=[48.35, 48.23, 1.75, 0.0, 0.0, 0.0], state=[32] + [0] * 5)


def test_unitree_hg_status():
    emb = load_embodiment(G1)
    robot = get_body("unitree_sdk2")(emb, msg_family="hg")
    motors = [_hg_motor(30 + i) for i in range(35)]
    motors[3] = _hg_motor(55, fault=0x20)
    motors[13] = SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0, temperature=[0, 0],
                                 vol=0.0, motorstate=0)  # a locked waist slot reports nothing
    robot._low_state = SimpleNamespace(motor_state=motors)
    assert robot.get_status().battery is None  # no BMS message yet
    robot._bms_state, robot._board_state = _G1_BMS, _G1_BOARD

    status = robot.get_status()
    b = status.battery
    assert b.soc == 49 and b.health == 93 and b.cycles == 26
    assert b.voltage == pytest.approx(48.378) and b.current == pytest.approx(-2.239)
    assert b.temperature == 27
    assert status.joint_temperature.shape == (emb.num_dofs,)
    assert status.joint_temperature[0] == 30 and status.joint_temperature[3] == 55
    assert np.isnan(status.joint_temperature[13])
    assert status.joint_faults == {emb.joint_names[3]: 0x20}
    assert status.board_temperature == {"mainboard": 42.0}
    assert status.extra["battery_cell_voltage"] == pytest.approx([3.725, 3.726, 3.727])


def test_unitree_go_status_reads_battery_from_lowstate():
    emb = load_embodiment(H1)
    robot = get_body("unitree_sdk2")(emb, msg_family="go")
    motors = [SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0, temperature=40 + i) for i in range(20)]
    bms = SimpleNamespace(soc=80, cycle=12, bq_ntc=[30, 31], mcu_ntc=[33, 0])
    robot._low_state = SimpleNamespace(motor_state=motors, bms_state=bms,
                                       power_v=50.2, power_a=-3.5)
    status = robot.get_status()
    assert status.battery == BatteryState(soc=80, voltage=50.2, current=-3.5,
                                          temperature=33, cycles=12)
    # Slot 9 is unused on the legacy H1, so joint 9 reads slot 10.
    assert status.joint_temperature[8] == 48 and status.joint_temperature[9] == 50


def test_unitree_reports_the_quaternion_scalar_last():
    robot = get_body("unitree_sdk2")(load_embodiment(G1), msg_family="hg")
    imu = SimpleNamespace(quaternion=[0.9, 0.1, 0.2, 0.3], gyroscope=[0, 0, 0],
                          accelerometer=[0, 0, 9.8])
    robot._low_state = SimpleNamespace(motor_state=[_hg_motor(30) for _ in range(35)],
                                       imu_state=imu)
    # The SDK reports [w, x, y, z]; Observation is [x, y, z, w].
    assert np.allclose(robot.get_observation().base_orientation, [0.1, 0.2, 0.3, 0.9])
