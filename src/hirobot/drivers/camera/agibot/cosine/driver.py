"""AgiBot G1 (精灵 G01) camera driver over ``cosine_bus`` (registered as ``agibot_cosine``).

The G1's onboard computer publishes every camera on the vendor's ``cosine_bus``
image topics: ``/camera/head_color`` (JPEG, 1280x800), ``/camera/head_depth``
(uncompressed Z16, 1280x800), ``/camera/hand_left_color`` and
``/camera/hand_right_color`` (JPEG, 848x480), about 30 frames/s each. The
driver subscribes to one colour topic and, for an RGB-D camera, its depth
topic; it never publishes, so it is safe next to another controller.

Depth and colour of one capture share ``source_timestamp`` and
``sequence_number``. For each new colour frame the driver asks the depth
subscriber for the frame nearest to that ``source_timestamp`` and keeps it only
if it is the same capture (within ``max_pair_skew_s``). The depth packet can
arrive a few milliseconds after its colour packet, so a colour frame without a
partner is held back for up to ``pair_wait_s`` and then goes out without depth. Depth is converted to metres with
``depth_scale`` (RealSense Z16: 0.001 m per unit); 0 stays 0 (no reading).

``cosine_bus`` finds its peers through two environment variables that the
vendor's ``env.sh`` sets: ``LOCATOR_IP`` (this computer's address on the robot
network) and ``AORTA_DISCOVERY_URI``. Without them a subscriber waits forever,
so :meth:`start` checks for them first.
"""

from __future__ import annotations

import os
import time
from typing import Any, Optional, Tuple

import numpy as np

from hirobot.devices.camera import PollingCameraDriver, register_camera
from hirobot.loading.sdk import require_sdk

#: Environment variables ``cosine_bus`` needs (set by the vendor's env.sh).
REQUIRED_ENV = ("LOCATOR_IP", "AORTA_DISCOVERY_URI")


@register_camera("agibot_cosine")
class AgibotCosineCamera(PollingCameraDriver):
    """Frames from one AgiBot G1 camera topic, with depth for an RGB-D camera.

    Args:
        name: Camera name (the ``images`` / ``depth`` key).
        topic: Colour topic, e.g. ``/camera/head_color``.
        depth_topic: Depth topic of the same camera (``/camera/head_depth``),
            or ``None`` for a colour-only camera.
        depth_scale: Metres per raw Z16 unit.
        max_pair_skew_s: Largest ``source_timestamp`` difference at which a
            depth frame still counts as the colour frame's partner.
        pair_wait_s: How long a colour frame waits for its depth partner.
        queue: Frames each subscriber buffers (the newest is used).
        poll_hz: Reads per second; above the camera rate so no frame is
            missed (a read with no new frame is skipped).
    """

    def __init__(
        self,
        name: str,
        topic: str,
        depth_topic: Optional[str] = None,
        depth_scale: float = 0.001,
        max_pair_skew_s: float = 0.005,
        pair_wait_s: float = 0.02,
        queue: int = 4,
        poll_hz: float = 60.0,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        self.topic = topic
        self.depth_topic = depth_topic
        self.depth_scale = float(depth_scale)
        self._max_skew_ns = int(max_pair_skew_s * 1e9)
        self._pair_wait_s = float(pair_wait_s)
        self._waiting: Optional[Tuple[int, float]] = None  # (sequence, first seen)
        self._queue = int(queue)
        self._bus: Any = None
        self._cv2: Any = None
        self._color: Any = None
        self._depth_sub: Any = None
        self._last_seq: Optional[int] = None
        #: Frames that went out without depth because no partner was found.
        self.unpaired = 0

    def _open(self) -> None:
        missing = [k for k in REQUIRED_ENV if not os.environ.get(k)]
        if missing:
            raise RuntimeError(
                f"AgibotCosineCamera: {', '.join(missing)} not set; cosine_bus needs them to find "
                "the robot's publishers. Source the vendor environment first: "
                "source <vendor app dir>/env.sh <vendor app dir>/lib (docs/setup/agibot_g1.md)")
        require_sdk(__package__, "AgibotCosineCamera")
        try:
            import cosine_bus
            import cv2
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "AgibotCosineCamera needs cosine_bus (preinstalled on the G1's onboard computer) "
                "and opencv-python (pip install 'hirobot[camera]')") from exc
        self._bus = cosine_bus
        self._cv2 = cv2
        best_effort = cosine_bus.QosReliability.BEST_EFFORT
        self._color = cosine_bus.image_subscriber(self.topic, best_effort, self._queue)
        if self.depth_topic:
            self._depth_sub = cosine_bus.image_subscriber(self.depth_topic, best_effort,
                                                          self._queue)

    def _grab(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        packet = self._color.read_latest_frame()
        if packet is None or packet.type != self._bus.PacketType.IMAGE:
            return None
        if packet.sequence_number == self._last_seq:
            return None
        depth = None
        if self._depth_sub is not None:
            depth = self._pair_depth(packet.source_timestamp)
            if depth is None:
                seq = packet.sequence_number
                if self._waiting is None or self._waiting[0] != seq:
                    self._waiting = (seq, time.monotonic())
                if time.monotonic() - self._waiting[1] < self._pair_wait_s:
                    return None  # partner may still be on its way
                self.unpaired += 1
        self._waiting = None
        image = self._decode_color(packet)
        self._last_seq = packet.sequence_number
        return image, depth

    def _decode_color(self, packet: Any) -> np.ndarray:
        bus, cv2 = self._bus, self._cv2
        data = np.asarray(packet.get_image_data(), dtype=np.uint8)
        h, w = packet.image_height, packet.image_width
        if packet.encoding_format in (bus.EncodingFormat.JPEG, bus.EncodingFormat.PNG):
            bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError(f"{self.topic}: frame {packet.sequence_number} does not decode")
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if packet.color_format == bus.ColorFormat.RGB:
            return data.reshape(h, w, 3).copy()
        if packet.color_format == bus.ColorFormat.BGR:
            return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_BGR2RGB)
        raise RuntimeError(f"{self.topic}: unsupported colour packet "
                           f"({packet.encoding_format}, {packet.color_format})")

    def _pair_depth(self, source_ns: int) -> Optional[np.ndarray]:
        packet = self._depth_sub.read_frame_nearest(source_ns)
        if packet is None or packet.type != self._bus.PacketType.IMAGE:
            return None
        if abs(packet.source_timestamp - source_ns) > self._max_skew_ns:
            return None
        if packet.color_format != self._bus.ColorFormat.RS2_FORMAT_Z16:
            raise RuntimeError(f"{self.depth_topic}: expected Z16 depth, got "
                               f"{packet.color_format}")
        raw = np.frombuffer(packet.get_image_data(), dtype=np.uint16)
        raw = raw.reshape(packet.image_height, packet.image_width)
        return raw.astype(np.float32) * np.float32(self.depth_scale)

    def _close(self) -> None:
        self._color = None
        self._depth_sub = None
        self._last_seq = None
        self._waiting = None
