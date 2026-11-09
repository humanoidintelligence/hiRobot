"""Robot.get_status: the default and the assembly. The Unitree mapping: tests/drivers/body/unitree/."""

from types import SimpleNamespace

import numpy as np

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import RobotAssembly, RobotStatus, get_body, load_embodiment

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


def test_default_status_is_empty():
    robot = get_body("toy")(load_embodiment(G1))
    status = robot.get_status()
    assert isinstance(status, RobotStatus)
    assert status.timestamp is not None
    assert status.battery is None and status.joint_temperature is None
    assert status.joint_faults == {} and status.board_temperature == {}


def test_assembly_pads_hand_joints():
    emb = load_embodiment("unitree_g1_dex3")
    body_emb = emb.model_copy(update={"joints": emb.joints[:29]})
    body = get_body("unitree_sdk2")(body_emb, msg_family="hg")
    body._low_state = SimpleNamespace(motor_state=[_hg_motor(35) for _ in range(35)])
    hand_names = emb.joint_names[29:]
    hands = {"left": SimpleNamespace(joint_names=hand_names[:7], interface="joints"),
             "right": SimpleNamespace(joint_names=hand_names[7:], interface="joints")}
    status = RobotAssembly(body, end_effectors=hands, embodiment=emb).get_status()
    assert status.joint_temperature.shape == (emb.num_dofs,)
    assert np.all(status.joint_temperature[:29] == 35)
    assert np.all(np.isnan(status.joint_temperature[29:]))


