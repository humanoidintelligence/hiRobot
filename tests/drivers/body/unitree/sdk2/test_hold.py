"""``unitree_sdk2`` hold() under builtin_skills."""

from __future__ import annotations


def test_unitree_hold_with_builtin_skills_is_a_noop():
    from hirobot.drivers.body.unitree.sdk2 import UnitreeRobot

    robot = UnitreeRobot.__new__(UnitreeRobot)
    robot._builtin_skills = "g1"
    UnitreeRobot.hold(robot)  # would raise SafetyError if it sent joint targets
