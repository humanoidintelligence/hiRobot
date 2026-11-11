"""Simulator drivers: reset(seed)."""

from __future__ import annotations

from hirobot.drivers.sim.toy import ToyRobot
from hirobot.core.embodiment import Embodiment, Joint


def test_toy_sim_reset_takes_a_seed():
    robot = ToyRobot(Embodiment(name="t", joints=[Joint(name="a")]))
    robot.connect()
    robot.reset(seed=3)  # ignored, must not raise
    robot.reset(3)
