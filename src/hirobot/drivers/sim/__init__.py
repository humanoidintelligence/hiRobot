"""Simulator drivers: simulated robots behind the Robot contract.

- ``toy`` (:mod:`.toy`): a minimal in-process integrator, no dependencies.
- ``mujoco``: MuJoCo physics, also the twin of a real robot (see
  :mod:`hirobot.twin`).

Benchmark environments live in their own packages and register through the
``hirobot.drivers`` entry point.

MuJoCo is imported lazily, so importing this package (which registers the
drivers) does not need it.
"""

from hirobot.drivers.sim.mujoco import MujocoRobot
from hirobot.drivers.sim.toy import ToyRobot

__all__ = [
    "ToyRobot",
    "MujocoRobot",
]
