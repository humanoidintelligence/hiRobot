"""Hardware and simulator drivers, grouped by component.

Real hardware is laid out as ``drivers/<component>/<SDK vendor>/<SDK or protocol>/``;
each such directory has a ``driver.py`` and a ``manifest.yaml`` naming its SDK
and the models and SDK versions it has run on (see :mod:`hirobot.loading.sdk`).

- :mod:`hirobot.drivers.body`: real robot bodies (vendor SDKs).
- :mod:`hirobot.drivers.sim`: simulated robots (toy integrator, MuJoCo).
- :mod:`hirobot.drivers.camera`: camera drivers that fill
  ``Observation.images`` independently of the body driver.
- :mod:`hirobot.drivers.end_effector`: dexterous hands that drive an end-effector's
  finger joints independently of the body driver.
- :mod:`hirobot.drivers.audio`: microphones and speakers attached to the robot.

Importing this package registers the built-in drivers with the registry.
"""

from hirobot.drivers import audio, camera, end_effector  # noqa: F401 (register them)
from hirobot.drivers.audio import UnitreeG1Microphone, UnitreeG1Speaker
from hirobot.drivers.body import (
    AgibotG1Robot,
    AgibotX1Robot,
    FourierRobot,
    UnitreeRobot,
    URRobot,
)
from hirobot.drivers.end_effector import UnitreeDex3Hand
from hirobot.drivers.sim import MujocoRobot, ToyRobot

__all__ = [
    "ToyRobot",
    "MujocoRobot",
    "UnitreeRobot",
    "FourierRobot",
    "URRobot",
    "AgibotX1Robot",
    "AgibotG1Robot",
    "UnitreeDex3Hand",
    "UnitreeG1Microphone",
    "UnitreeG1Speaker",
]
