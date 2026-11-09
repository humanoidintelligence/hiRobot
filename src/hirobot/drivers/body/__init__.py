"""Real-robot body drivers: vendor SDKs behind the Robot contract.

Each driver wraps one vendor SDK (Unitree, Fourier, AgiBot, Franka, UR) behind
:class:`~hirobot.robot.Robot`. SDKs are imported lazily on ``connect()``, so
importing this package (which registers the drivers) needs none of them.
"""

from hirobot.drivers.body.agibot.aimrt import AgibotX1Robot
from hirobot.drivers.body.agibot.gdk import AgibotG1Robot
from hirobot.drivers.body.fourier.grx import FourierRobot
from hirobot.drivers.body.franka.libfranka import FrankaRobot
from hirobot.drivers.body.unitree.sdk2 import UnitreeRobot
from hirobot.drivers.body.ur.rtde import URRobot

__all__ = ["UnitreeRobot", "FourierRobot", "URRobot", "FrankaRobot", "AgibotX1Robot", "AgibotG1Robot"]
