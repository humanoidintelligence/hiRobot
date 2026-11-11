"""``unitree_videohub``: built for real Unitree drivers, sharing the body's NIC."""

from __future__ import annotations

from hirobot import RobotAssembly, make_robot
from hirobot.drivers.camera import UnitreeVideohubCamera

G1 = "unitree_g1"


def test_make_robot_attaches_videohub_for_real_driver():
    robot = make_robot(G1, overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth9"}}})
    assert isinstance(robot, RobotAssembly)
    assert type(robot.body).__name__ == "UnitreeRobot"
    cam = robot.cameras["head"]
    assert isinstance(cam, UnitreeVideohubCamera)
    assert cam._network_interface == "eth9"  # shares the body's DDS NIC
    assert robot.safety is not None
