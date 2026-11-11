"""A camera that cycles through frames held in memory (tests, demos).

Declared like any camera driver::

    cameras:
      - name: head
        driver:
          type: array
          fps: 10

With no ``frames`` it shows a single grey 64x64 frame, so a config can name
it without any data. Code builds it with real frames directly.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence

import numpy as np

from hirobot.devices.camera import CameraDriver, Frame, register_camera
from hirobot.core.stamps import Stamp


@register_camera("array")
class ArrayCamera(CameraDriver):
    """Cycle through ``frames`` at ``fps``, on the host clock.

    Args:
        name: Camera name.
        frames: HxWx3 uint8 frames, shown in order and then from the start again.
        fps: Frames per second.
        advance_on_read: Test switch. Each :meth:`latest` returns the next frame
            instead of the one due at the current time, so a test steps through
            the frames without waiting. A live camera never does this.
    """

    #: Not real hardware: ``make_robot`` attaches it to simulated drivers too.
    simulated = True

    def __init__(
        self,
        name: str,
        frames: Optional[Sequence[np.ndarray]] = None,
        fps: float = 10.0,
        advance_on_read: bool = False,
    ) -> None:
        super().__init__(name)
        if frames is None:
            frames = [np.full((64, 64, 3), 128, dtype=np.uint8)]
        if len(frames) == 0:
            raise ValueError("ArrayCamera needs at least one frame")
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        self.frames = [np.asarray(f, dtype=np.uint8) for f in frames]
        self.fps = float(fps)
        self.advance_on_read = advance_on_read
        self._start: Optional[float] = None
        self._next = 0
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._start is None:
            self._start = time.monotonic()
            self._next = 0

    def latest(self) -> Optional[Frame]:
        if self._start is None:
            return None
        with self._lock:
            if self.advance_on_read:
                index = self._next
                self._next += 1
            else:
                index = int((time.monotonic() - self._start) * self.fps)
        # Stamp it as if it came at its slot in the stream; frame numbers start at 1.
        return self.frames[index % len(self.frames)], Stamp(self._start + index / self.fps,
                                                            index + 1)

    def stop(self) -> None:
        self._start = None
