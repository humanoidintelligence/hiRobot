"""Real-robot body drivers: vendor SDKs behind the Robot contract.

Each driver wraps one vendor SDK (Unitree, Fourier, AgiBot, AgileX, Franka, UR), a robot server
(DROID) or a generic protocol (ROS 2) behind
:class:`~hirobot.core.robot.Robot`. SDKs are imported lazily on ``connect()``, so
importing this package (which registers the drivers) needs none of them.
"""

from hirobot.drivers.body.agibot.gdk import AgibotG1Robot
from hirobot.drivers.body.agilex.piper_sdk import PiperRobot
from hirobot.drivers.body.fourier.grx import FourierRobot
from hirobot.drivers.body.franka.droid import FrankaDroidRobot
from hirobot.drivers.body.generic.ros2 import Ros2Robot
from hirobot.drivers.body.trossen.dynamixel import WidowXRobot
from hirobot.drivers.body.unitree.sdk2 import UnitreeRobot
from hirobot.drivers.body.ur.rtde import URRobot

__all__ = ["UnitreeRobot", "FourierRobot", "URRobot", "FrankaDroidRobot", "AgibotG1Robot", "PiperRobot", "WidowXRobot",
           "Ros2Robot"]
