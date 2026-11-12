"""Intel RealSense camera driver over ``pyrealsense2`` (registered as ``intel_realsense``).

Wraps a RealSense device's ``pipeline`` behind the
:class:`~hirobot.devices.camera.PollingCameraDriver` contract. One driver drives
one physical camera, selected by ``serial`` (use the serial so a fixed camera
keeps its role when others are plugged in). It streams the colour sensor and,
for an RGB-D camera (``depth: true``), the depth sensor aligned to colour so the
two images share one pixel grid.

``pyrealsense2.align`` maps each depth frame into the colour frame, so
``Observation.depth`` lines up with ``Observation.images``. Depth comes out of
the SDK as uint16 in device units; the depth sensor's ``depth_scale``
(metres per unit, read from the device at ``_open``) converts it to metres, and
0 stays 0 (no reading). ``pyrealsense2`` is imported lazily in ``_open``;
install it with ``pip install pyrealsense2``.

Run on a D435i (``pyrealsense2`` 2.59.1) and an L515 (``pyrealsense2`` 2.54.2;
2.59.1 does not list the L515). The driver does not use
the IMU. Tested against a fake ``pyrealsense2`` in the contract test; see
``docs/setup/ur.md`` for real-camera notes.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

import numpy as np

from hirobot.devices.camera import PollingCameraDriver, register_camera
from hirobot.loading.sdk import require_sdk


@register_camera("intel_realsense")
class RealSenseCamera(PollingCameraDriver):
    """Frames from one Intel RealSense camera, with aligned depth for an RGB-D camera.

    Args:
        name: Camera name (the ``images`` / ``depth`` key).
        serial: Device serial number to open. ``None`` opens the first device
            the SDK enumerates (fine when only one is attached).
        width / height: Colour (and depth) stream size in pixels.
        fps: Stream frame rate.
        depth: Stream the depth sensor and align it to colour; the frame then
            carries a depth image in metres.
        depth_width / depth_height: Depth stream size, when it differs from
            colour (``None`` uses ``width`` / ``height``). The aligned depth
            image still comes out at the colour size. An L515 on USB 2 offers
            depth only at 320x240.
        timeout_ms: How long one ``wait_for_frames`` blocks.
        poll_hz: Grab attempts per second; at or above ``fps``.
        network_interface: Ignored (``make_robot`` fills it in for every
            hardware camera); RealSense is a local USB device.
    """

    def __init__(
        self,
        name: str,
        serial: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        depth: bool = False,
        depth_width: Optional[int] = None,
        depth_height: Optional[int] = None,
        timeout_ms: int = 1000,
        poll_hz: float = 60.0,
        network_interface: Optional[str] = None,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        self.serial = None if serial is None else str(serial)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.depth = bool(depth)
        self.depth_width = self.width if depth_width is None else int(depth_width)
        self.depth_height = self.height if depth_height is None else int(depth_height)
        self._timeout_ms = int(timeout_ms)
        self._rs: Any = None
        self._pipeline: Any = None
        self._align: Any = None
        self._depth_scale: float = 0.001
        self._last_frame_number: Optional[int] = None

    def _open(self) -> None:
        require_sdk(__package__, "RealSenseCamera")
        try:
            import pyrealsense2 as rs
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "RealSenseCamera needs the pyrealsense2 package; install it with "
                "'pip install pyrealsense2'.") from exc
        self._rs = rs
        config = rs.config()
        if self.serial is not None:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        if self.depth:
            config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height,
                                 rs.format.z16, self.fps)
        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(config)
        if self.depth:
            # Align depth into the colour frame so the two images share a pixel grid.
            self._align = rs.align(rs.stream.color)
            self._depth_scale = float(
                profile.get_device().first_depth_sensor().get_depth_scale())

    def _grab(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        try:
            frames = self._pipeline.wait_for_frames(self._timeout_ms)
        except RuntimeError as exc:  # SDK raises on timeout
            raise TimeoutError(
                f"camera {self.name!r}: no frames within {self._timeout_ms} ms; "
                "is the RealSense connected?") from exc
        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            return None
        # Skip a frame already handed out (wait_for_frames can repeat under load).
        number = color.get_frame_number()
        if number == self._last_frame_number:
            return None
        self._last_frame_number = number
        image = np.asanyarray(color.get_data())  # HxWx3 uint8 RGB (rs.format.rgb8)
        image = np.ascontiguousarray(image)
        depth = None
        if self.depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                raw = np.asanyarray(depth_frame.get_data())  # HxW uint16
                depth = raw.astype(np.float32) * np.float32(self._depth_scale)
        return image, depth

    def _close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except RuntimeError:  # pragma: no cover - already stopped
                pass
            self._pipeline = None
        self._align = None
        self._last_frame_number = None

