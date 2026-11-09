"""Hand drivers: dexterous hands commanded separately from the body.

A hand's finger joints are ordinary embodiment joints; an end-effector lists
them and names its driver (see :mod:`hirobot.end_effector`). Vendor SDKs
are imported lazily on ``connect()``. Layout: ``hands/<SDK vendor>/<SDK or protocol>/``.
"""

from hirobot.drivers.end_effector.inspire.sdkpy import InspireSdkpyHand
from hirobot.drivers.end_effector.unitree.sdk2_handcmd import UnitreeDex3Hand
from hirobot.drivers.end_effector.unitree.sdk2_motorcmds import UnitreeMotorCmdsHand

__all__ = ["UnitreeDex3Hand", "UnitreeMotorCmdsHand", "InspireSdkpyHand"]
