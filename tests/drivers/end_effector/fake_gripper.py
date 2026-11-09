"""A gripper on the ``opening`` interface, for tests: no SDK, moves instantly.

No built-in driver has the ``opening`` interface yet (Robotiq and Franka Hand
come later), so this stands in for one. :func:`registered` puts it in the
end-effector registry for one test only.
"""

from __future__ import annotations

import contextlib
import time
from typing import List, Optional

from hirobot.end_effector import (
    END_EFFECTORS,
    EndEffectorDriver,
    OpeningState,
    check_opening,
)
from hirobot.stamps import Stamp

NAME = "test_fake_gripper"


class FakeGripper(EndEffectorDriver):
    interfaces = ("opening",)

    def __init__(self, name, joints, start: float = 1.0, read_only: bool = False,
                 network_interface: Optional[str] = None) -> None:
        super().__init__(name, joints, read_only=read_only)
        self.opening = float(start)
        self.sent: List[float] = []
        self.connected = False
        self.damped = 0
        self._seq = 0

    def connect(self) -> None:
        self.connected = True

    def get_state(self) -> OpeningState:
        if not self.connected:
            raise RuntimeError(f"{self.name}: not connected")
        self._seq += 1
        return OpeningState(self.opening, Stamp(time.monotonic(), self._seq),
                            extra={"object": 0.0})

    def command_opening(self, opening: float) -> None:
        if self.read_only:
            raise RuntimeError(f"{self.name}: read_only")
        self.opening = check_opening(opening)
        self.sent.append(self.opening)

    def reset(self) -> None:
        if self.read_only:
            raise RuntimeError(f"{self.name}: read_only")

    def damp(self) -> None:
        if not self.read_only:
            self.damped += 1


@contextlib.contextmanager
def registered():
    """Register :class:`FakeGripper` as :data:`NAME` for the block, then remove it."""
    END_EFFECTORS.register(NAME)(FakeGripper)
    try:
        yield FakeGripper
    finally:
        END_EFFECTORS._entries.pop(NAME, None)
