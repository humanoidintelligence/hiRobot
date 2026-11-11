"""Command watchdog: stop a velocity command the caller stopped refreshing.

A driver that streams joint or base velocities to the robot keeps executing
the last one when the caller hangs or dies. :class:`CommandWatchdog` notes each
command (:meth:`CommandWatchdog.feed`, called from the driver's
``_send_action``) and, once ``timeout_s`` passes with no new one in a watched
mode, calls the driver's stop callback once and logs a warning. It does not
enter the fault state; the next command runs as usual.

By default only ``VELOCITY`` and ``BASE_VELOCITY`` are watched: a position
target is a fixed point the robot reaches and stays at, and watching it would
trip low-rate callers. A driver passes ``modes`` to watch more.

Hardware drivers run the check on a timer thread (``thread=True``) against
``time.monotonic()``. Simulators advance only when called, so they pass their
own clock and call :meth:`CommandWatchdog.check` from ``_idle_step``.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Iterable, Optional

from hirobot.core.types import ControlMode

logger = logging.getLogger(__name__)

#: Seconds without a new velocity command before a driver stops the motion.
DEFAULT_COMMAND_TIMEOUT_S = 0.2

#: Modes watched by default.
VELOCITY_MODES = (ControlMode.VELOCITY, ControlMode.BASE_VELOCITY)


class CommandWatchdog:
    """Call ``on_timeout(mode)`` once when a watched command goes stale.

    Args:
        name: Shown in the warning (the driver's class name).
        timeout_s: Seconds without a new command; ``None`` or ``<= 0`` turns
            the watchdog off.
        on_timeout: Stops the motion (zero velocity, ``speedStop``). Called
            once per stale command, from the timer thread or from :meth:`check`.
        modes: Modes that arm the watchdog. A command in any other mode
            replaces the watched one and disarms it.
        clock: Seconds; ``time.monotonic`` by default, the simulated time in a
            simulator.
        thread: Run :meth:`check` on a daemon timer thread (hardware drivers).
        lock: Held around ``on_timeout`` so it does not interleave with a
            command being sent (the driver's command lock).
    """

    def __init__(
        self,
        name: str,
        timeout_s: Optional[float],
        on_timeout: Callable[[ControlMode], None],
        *,
        modes: Iterable[ControlMode] = VELOCITY_MODES,
        clock: Callable[[], float] = time.monotonic,
        thread: bool = False,
        lock: Optional[threading.RLock] = None,
    ) -> None:
        self.name = name
        self.timeout_s = float(timeout_s) if timeout_s and timeout_s > 0 else None
        self.modes = frozenset(modes)
        self._on_timeout = on_timeout
        self._clock = clock
        self._use_thread = thread
        self._lock = lock if lock is not None else threading.RLock()
        self._armed: Optional[tuple] = None  # (mode, time of the last command)
        self._state = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        #: How many times the watchdog stopped a stale command (for tests, status).
        self.timeouts = 0

    @property
    def enabled(self) -> bool:
        return self.timeout_s is not None

    def feed(self, mode: ControlMode) -> None:
        """Note a command just sent in ``mode``."""
        if not self.enabled:
            return
        with self._state:
            self._armed = (mode, self._clock()) if mode in self.modes else None
        if self._armed is not None and self._use_thread:
            self._start()

    def disarm(self) -> None:
        """Forget the last command (the driver stopped the motion itself)."""
        with self._state:
            self._armed = None

    def check(self) -> bool:
        """Stop the motion if the watched command is stale; ``True`` if it did."""
        with self._state:
            armed = self._armed
            if armed is None or self._clock() - armed[1] <= self.timeout_s:
                return False
            self._armed = None
        with self._lock:
            self.timeouts += 1
            logger.warning("%s: no new %s command for %.2f s; stopping the motion",
                           self.name, armed[0].value, self.timeout_s)
            try:
                self._on_timeout(armed[0])
            except Exception:  # noqa: BLE001 - keep watching; the error is logged
                logger.exception("%s: stopping a stale %s command failed",
                                 self.name, armed[0].value)
        return True

    def close(self) -> None:
        """Stop the timer thread and disarm (``disconnect``)."""
        self.disarm()
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"watchdog-{self.name}",
                                        daemon=True)
        self._thread.start()

    def _run(self) -> None:
        period = min(self.timeout_s / 4.0, 0.05)
        while not self._stop.wait(period):
            self.check()


__all__ = ["CommandWatchdog", "DEFAULT_COMMAND_TIMEOUT_S", "VELOCITY_MODES"]
