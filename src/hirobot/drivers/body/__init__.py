"""Real-robot body drivers: vendor SDKs behind the Robot contract.

Each driver wraps one vendor SDK (Unitree, Fourier, AgiBot, AgileX, Franka, UR) or a robot server (DROID) behind
:class:`~hirobot.core.robot.Robot`. SDKs are imported lazily on ``connect()``, so
importing this package (which registers the drivers) needs none of them.
"""

from hirobot.drivers.body.agibot.aimrt import AgibotX1Robot
from hirobot.drivers.body.agibot.gdk import AgibotG1Robot
from hirobot.drivers.body.agilex.piper_sdk import PiperRobot
from hirobot.drivers.body.fourier.grx import FourierRobot
from hirobot.drivers.body.franka.droid import FrankaDroidRobot
from hirobot.drivers.body.unitree.sdk2 import UnitreeRobot
from hirobot.drivers.body.ur.rtde import URRobot

__all__ = ["UnitreeRobot", "FourierRobot", "URRobot", "FrankaDroidRobot", "AgibotX1Robot", "AgibotG1Robot", "PiperRobot"]
