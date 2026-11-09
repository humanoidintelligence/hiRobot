"""When each data source's reading arrived: :class:`Stamp` and helpers.

An observation is a snapshot of several sources that update on their own
clocks: the body's joint state, each hand, each camera. ``Observation.stamps``
maps every source that contributed to the snapshot to the :class:`Stamp` of
the reading it contributed. Source names are ``body`` for the body driver, the
end-effector name for a hand or gripper driver, and the camera name for a
camera; a robot group prefixes them with the member name (``left/body``).

Times are on the robot's clock, :meth:`hirobot.robot.Robot.now`: the host's
``time.monotonic()`` on hardware, simulated time in a simulator. So
``robot.now() - obs.stamps["head"].t`` is the age of the head camera's frame on
either. Monotonic times only compare within one host; send ages, not stamps, to
another machine.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Callable, List, Mapping, NamedTuple, Optional, Union

if TYPE_CHECKING:
    from hirobot.embodiment import Embodiment
    from hirobot.types import Observation


class Stamp(NamedTuple):
    """When a source's reading arrived, and which reading it is.

    Attributes:
        t: Arrival time in seconds on the robot's clock (:meth:`Robot.now`).
        seq: The source's reading number: 1 for its first reading, one more
            for each new one. Two observations with the same ``seq`` for a
            source carry the same reading of it.
    """

    t: float
    seq: int


class StampCounter:
    """Hands out the :class:`Stamp` of each new reading of one source.

    Call :meth:`tick` where a new reading arrives (an SDK callback, a camera
    grab); it records the time and the next sequence number. Thread-safe.

    Args:
        clock: Where ``t`` comes from when :meth:`tick` gets none. The default
            is ``time.monotonic``, the hardware robot clock.
    """

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._seq = 0
        self._last: Optional[Stamp] = None

    def tick(self, t: Optional[float] = None) -> Stamp:
        """Stamp a new reading that arrived at ``t`` (default: now on ``clock``)."""
        with self._lock:
            self._seq += 1
            self._last = Stamp(self._clock() if t is None else float(t), self._seq)
            return self._last

    @property
    def last(self) -> Optional[Stamp]:
        """The newest stamp handed out, or ``None`` before the first reading."""
        return self._last


def stale_sources(
    obs: "Observation",
    now: float,
    max_age: Union[None, float, Mapping[str, float]] = None,
    embodiment: Optional["Embodiment"] = None,
    periods: float = 3.0,
) -> List[str]:
    """Sources of ``obs`` whose reading is older than allowed at time ``now``.

    The allowed age of a source is, in order: ``max_age[source]`` (a mapping)
    or ``max_age`` (one number for every source); else ``periods`` nominal
    periods from ``embodiment``: ``1 / camera.fps`` for a camera that declares
    ``fps``, ``1 / control_hz`` for the body and the end-effectors (their state
    arrives at least that often on every supported robot). Sources with no
    allowed age are never stale. ``now`` is ``robot.now()``.

    Whether a reading is too old depends on the caller (a whole-body controller
    tolerates a few milliseconds, a VLA a few hundred), so hiRobot never marks
    or drops a stale source itself.
    """
    limits = {}
    if embodiment is not None:
        if embodiment.control_hz:
            period = periods / embodiment.control_hz
            limits["body"] = period
            for ee in embodiment.end_effectors:
                limits[ee.name] = period
        for cam in embodiment.cameras:
            if cam.fps:
                limits[cam.name] = periods / cam.fps
    stale = []
    for name, stamp in obs.stamps.items():
        if isinstance(max_age, Mapping):
            limit = max_age.get(name, limits.get(name))
        elif max_age is not None:
            limit = float(max_age)
        else:
            limit = limits.get(name)
        if limit is not None and now - stamp.t > limit:
            stale.append(name)
    return stale
