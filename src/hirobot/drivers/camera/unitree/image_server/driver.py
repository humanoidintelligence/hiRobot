"""Frames from Unitree's teleoperation ``image_server`` (JPEG over ZeroMQ PUB).

Unitree's ``image_server.py`` (avp_teleoperate / xr_teleoperate) runs on the
computer the cameras are plugged into and publishes each frame as one ZeroMQ
message holding a single JPEG, on ``tcp://*:5555``. Several cameras are
concatenated side by side into one image; ``crop`` takes one of them out. Its
``Unit_Test`` mode prefixes ``struct.pack('dI', timestamp, frame_id)`` (12
bytes); set ``header: true`` for it. ``scripts/camera_image_server.py`` serves
one OpenCV camera in the same format, for a computer without the Unitree server.

The driver only subscribes, so it is safe next to a ``read_only`` body.
"""

from __future__ import annotations

import struct
from typing import Optional

import numpy as np

from hirobot.devices.camera import PollingCameraDriver, register_camera
from hirobot.loading.sdk import require_sdk

#: ``struct.pack('dI', ...)``: a float64 and a uint32, 12 bytes.
_HEADER = struct.calcsize("dI")


@register_camera("unitree_image_server")
class UnitreeImageServerCamera(PollingCameraDriver):
    """The newest frame published by a Unitree ``image_server``.

    Args:
        name: Camera name (the ``images`` key).
        address: The publisher, e.g. ``tcp://192.168.123.162:5555``.
        header: Each message starts with ``struct.pack('dI', stamp, frame_id)``
            (``image_server.py`` with ``Unit_Test=True``).
        crop: Optional ``[x, y, width, height]`` taken from the decoded frame,
            for a publisher that concatenates several cameras side by side.
        timeout_s: How long one grab waits for a message.
        poll_hz: Grab attempts per second.
        network_interface: Ignored (``make_robot`` fills it in for every
            hardware camera); ZeroMQ routes by address.
    """

    def __init__(
        self,
        name: str,
        address: str,
        header: bool = False,
        crop: Optional[list] = None,
        timeout_s: float = 1.0,
        poll_hz: float = 60.0,
        network_interface: Optional[str] = None,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        if crop is not None and len(crop) != 4:
            raise ValueError(f"camera {name!r}: crop is [x, y, width, height], got {crop!r}")
        self.address = address
        self._header = bool(header)
        self._crop = None if crop is None else [int(v) for v in crop]
        self._timeout_ms = int(timeout_s * 1000)
        self._ctx = None
        self._sock = None
        self._cv2 = None
        self._zmq = None
        #: Publisher's stamp and frame id of the newest frame (``header: true`` only).
        self.remote_stamp: Optional[float] = None
        self.remote_frame_id: Optional[int] = None

    def _open(self) -> None:
        require_sdk(__package__, "UnitreeImageServerCamera")
        import cv2
        import zmq

        self._cv2, self._zmq = cv2, zmq
        self._ctx = zmq.Context()
        sock = self._ctx.socket(zmq.SUB)
        # Keep only the newest message: a slow consumer never sees stale frames.
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(self.address)
        self._sock = sock

    def _grab(self) -> np.ndarray:
        try:
            msg = self._sock.recv()
        except self._zmq.Again:
            raise TimeoutError(
                f"no message from {self.address} within {self._timeout_ms} ms; "
                "is the image server running?") from None
        if self._header:
            if len(msg) < _HEADER:
                raise RuntimeError(f"message of {len(msg)} bytes is shorter than the header")
            self.remote_stamp, self.remote_frame_id = struct.unpack("dI", msg[:_HEADER])
            msg = msg[_HEADER:]
        bgr = self._cv2.imdecode(np.frombuffer(msg, np.uint8), self._cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"{self.address} sent a message that is not a decodable JPEG")
        if self._crop is not None:
            x, y, w, h = self._crop
            bgr = bgr[y:y + h, x:x + w]
            if bgr.shape[0] != h or bgr.shape[1] != w:
                raise RuntimeError(f"crop {self._crop} is outside the {bgr.shape[1]}x"
                                   f"{bgr.shape[0]} frame")
        return np.ascontiguousarray(self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB))

    def _close(self) -> None:
        if self._sock is not None:
            self._sock.close(0)
            self._sock = None
        if self._ctx is not None:
            self._ctx.term()
            self._ctx = None
