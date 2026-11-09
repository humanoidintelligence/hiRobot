"""The camera-driver contract and its registry.

A :class:`CameraDriver` delivers frames from one camera. Camera drivers are
separate from the body driver: ``make_robot`` builds one per camera declared
with a ``driver:`` in the embodiment config and attaches them to the robot, which
adds the latest frame of each to ``Observation.images`` and its
:class:`~hirobot.stamps.Stamp` to ``Observation.stamps`` (see
:class:`~hirobot.robot.Robot`). An
RGB-D camera (``depth: true``) also implements :meth:`CameraDriver.latest_rgbd`,
whose depth image goes to ``Observation.depth``.

Most cameras deliver frames on their own clock, so the usual implementation
runs a background thread that keeps only the newest frame
(:class:`PollingCameraDriver`); :meth:`CameraDriver.latest` then never blocks
the control loop.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from hirobot.registry import Registry
from hirobot.stamps import Stamp, StampCounter

#: A frame and its :class:`~hirobot.stamps.Stamp` (receive time on
#: ``time.monotonic()``, frame number).
Frame = Tuple[np.ndarray, Stamp]
#: A colour image, the depth image registered to it (HxW float32, metres, or
#: ``None`` when no depth came with it) and their common stamp.
RgbdFrame = Tuple[np.ndarray, Optional[np.ndarray], Stamp]

class CameraDriver(ABC):
    """One camera stream.

    Args:
        name: The camera's name; the key under which its frames appear in
            ``Observation.images``.
    """

    #: True for stand-ins that need no hardware (a video file, frames in memory).
    #: ``make_robot`` attaches only these to a simulated driver.
    simulated: bool = False

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def start(self) -> None:
        """Open the camera. Called once by the robot's ``connect()``."""

    @abstractmethod
    def latest(self) -> Optional[Frame]:
        """The newest frame (HxWx3 uint8 RGB) and its stamp, or ``None``.

        ``None`` means no frame has arrived yet. The stamp's ``seq`` goes up by
        one per new frame, so a caller sees when it gets the same frame again.
        Must not block.
        """

    def latest_rgbd(self) -> Optional[RgbdFrame]:
        """The newest colour image with its registered depth image, or ``None``.

        Both come from the same capture and share one stamp. Depth is HxW
        float32 in metres, 0 where the sensor has no reading. Drivers of RGB-D
        cameras override this; the default is :meth:`latest` without depth.
        Must not block.
        """
        frame = self.latest()
        return None if frame is None else (frame[0], None, frame[1])

    @abstractmethod
    def stop(self) -> None:
        """Release the camera. Safe to call more than once."""


class PollingCameraDriver(CameraDriver):
    """A camera driver that polls a blocking ``_grab()`` on a background thread.

    Subclasses implement :meth:`_open`, :meth:`_grab` and optionally
    :meth:`_close`. Only the newest frame is kept. An RGB-D subclass returns
    ``(image, depth)`` from :meth:`_grab`; :meth:`latest_rgbd` then gives the
    pair with one stamp. A grab that returns ``None`` (no new frame yet) is
    retried at the poll rate without counting as an error. A grab that raises is
    retried after ``retry_s``; the last error is kept on :attr:`error` and
    :attr:`frames` / :attr:`errors` count successes and failures.

    Args:
        name: Camera name.
        poll_hz: Upper bound on grab attempts per second. Set it at or above the
            camera's frame rate; a grab returning the same frame again is
            harmless but wastes bandwidth.
        retry_s: Pause after a failed grab.
    """

    def __init__(self, name: str, poll_hz: float = 30.0, retry_s: float = 0.2) -> None:
        super().__init__(name)
        self._period = 1.0 / poll_hz
        self._retry_s = retry_s
        self._lock = threading.Lock()
        self._frame: Optional[Frame] = None
        self._depth: Optional[np.ndarray] = None
        self._stamps = StampCounter()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[BaseException] = None
        self.frames = 0
        self.errors = 0

    def _open(self) -> None:
        """Connect to the camera. Runs in the caller's thread, so errors raise."""

    @abstractmethod
    def _grab(self) -> Any:
        """Block until a frame is available and return it as HxWx3 uint8 RGB.

        RGB-D drivers return ``(image, depth)`` with depth HxW float32 in metres
        (or ``None`` for a frame that came without depth). ``None`` means no new
        frame yet.
        """

    def _close(self) -> None:
        """Disconnect from the camera."""

    def start(self) -> None:
        if self._thread is not None:
            return
        self._open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=f"camera-{self.name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.time()
            try:
                grabbed = self._grab()
            except Exception as exc:  # noqa: BLE001 - a camera hiccup must not end the stream
                self.error = exc
                self.errors += 1
                self._stop.wait(self._retry_s)
                continue
            if grabbed is None:
                self._stop.wait(max(0.0, self._period - (time.time() - t0)))
                continue
            image, depth = grabbed if isinstance(grabbed, tuple) else (grabbed, None)
            stamp = self._stamps.tick()
            with self._lock:
                self._frame = (image, stamp)
                self._depth = depth
            self.frames += 1
            self._stop.wait(max(0.0, self._period - (time.time() - t0)))

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._frame

    def latest_rgbd(self) -> Optional[RgbdFrame]:
        with self._lock:
            if self._frame is None:
                return None
            return self._frame[0], self._depth, self._frame[1]

    def wait_for_frame(self, timeout: float) -> Frame:
        """Block until the first frame arrives (useful right after ``start()``)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.latest()
            if frame is not None:
                return frame
            time.sleep(0.01)
        detail = f": last error {self.error!r}" if self.error is not None else ""
        raise TimeoutError(f"camera {self.name!r} produced no frame within {timeout:.1f}s{detail}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
            self._close()


CAMERAS: Registry = Registry("camera driver", CameraDriver)
#: Class decorator that registers a :class:`CameraDriver` under a name.
register_camera = CAMERAS.register
#: Look up a registered camera-driver class by name.
get_camera = CAMERAS.get
#: Names of all registered camera drivers.
available_cameras = CAMERAS.available


def build_cameras(cameras: Sequence[Any], network_interface: Optional[str] = None,
                  sim_only: bool = False) -> Dict[str, CameraDriver]:
    """One camera driver per camera with a ``driver:`` spec, keyed by camera name.

    Args:
        cameras: :class:`~hirobot.embodiment.Camera` entries (an embodiment's
            ``cameras``, or a robot group's scene cameras).
        network_interface: Filled into a hardware camera's spec when it sets
            none (Unitree cameras share the body's DDS NIC).
        sim_only: Build only stand-in drivers (``simulated = True``) and skip
            hardware, so a simulated robot's config runs offline.
    """
    out: Dict[str, CameraDriver] = {}
    for cam in cameras:
        if not cam.driver:
            continue
        spec = dict(cam.driver)
        if "type" not in spec:
            raise ValueError(f"camera {cam.name!r}: driver needs a 'type'")
        cls = get_camera(spec.pop("type"))
        if sim_only and not cls.simulated:
            continue
        if network_interface is not None and not cls.simulated:
            spec.setdefault("network_interface", network_interface)
        out[cam.name] = cls(cam.name, **spec)
    return out
