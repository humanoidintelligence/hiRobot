"""Unitree ``videohub`` camera driver (e.g. the G1 head camera).

Unitree robots run a ``videohub`` service on the onboard computer that serves
the head camera as JPEG over the SDK2 RPC channel. On the G1 the service is
``/unitree/module/video_hub_pc4/videohub_pc4`` on PC2 and the camera is a
RealSense D435i colour stream at 1920x1080. The SDK only ships the client
under ``unitree_sdk2py.go2.video``, but the service name and API are shared,
so the same client works for the G1.

Each ``GetImageSample`` returns the newest JPEG. Measured on a G1 from a
desktop on the robot LAN: ~10 ms per request, ~130 KB per frame, and ~18 new
frames/s, so the default poll rate of 30 Hz fetches every frame.

The driver is read-only: it never publishes anything, so it is safe in the
``shadow`` sim mode and alongside another controller.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.camera import PollingCameraDriver, register_camera
from hirobot.drivers._sdk.unitree_sdk2 import init_channel
from hirobot.sdk import require_sdk

#: unitree_sdk2py.rpc.internal.RPC_ERR_CLIENT_SEND: the request found no subscriber
#: (the SDK waits up to the client timeout for one before giving up).
_RPC_ERR_CLIENT_SEND = 3102


@register_camera("unitree_videohub")
class UnitreeVideohubCamera(PollingCameraDriver):
    """Frames from a Unitree robot's ``videohub`` service.

    Args:
        name: Camera name (the ``images`` key).
        network_interface: NIC on the robot LAN (e.g. ``eth0``). If the body
            driver already initialised DDS, the SDK keeps that setting and
            this one is ignored, so give both the same NIC.
        domain_id: DDS domain (``0`` on hardware).
        timeout_s: RPC timeout per frame.
        poll_hz: Grab attempts per second (see :class:`PollingCameraDriver`).
    """

    def __init__(
        self,
        name: str,
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        timeout_s: float = 1.0,
        poll_hz: float = 30.0,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._timeout_s = timeout_s
        self._client = None
        self._cv2 = None

    def _open(self) -> None:
        require_sdk(__package__, "UnitreeVideohubCamera")
        try:
            import cv2
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "UnitreeVideohubCamera needs unitree_sdk2py (https://github.com/unitreerobotics/"
                "unitree_sdk2_python) and opencv-python (pip install 'hirobot[camera]')."
            ) from exc
        self._cv2 = cv2
        init_channel("UnitreeVideohubCamera", self._domain_id, self._network_interface)
        client = VideoClient()
        client.SetTimeout(self._timeout_s)
        client.Init()
        self._client = client

    def _grab(self) -> np.ndarray:
        code, data = self._client.GetImageSample()
        if code == _RPC_ERR_CLIENT_SEND:
            raise RuntimeError(
                f"videohub GetImageSample failed with code {code}: nothing subscribes to the "
                "videohub request topic, so the videohub service is not running (on the G1 it "
                "runs on PC2, 192.168.123.164)"
            )
        if code != 0 or not data:
            raise RuntimeError(f"videohub GetImageSample failed with code {code}")
        bgr = self._cv2.imdecode(np.frombuffer(bytes(data), np.uint8), self._cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError("videohub returned a frame that is not a decodable JPEG")
        return self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)

    def _close(self) -> None:
        self._client = None
