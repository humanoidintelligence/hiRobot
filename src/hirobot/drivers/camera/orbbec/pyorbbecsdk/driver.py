"""Orbbec camera driver over ``pyorbbecsdk`` 1.x (registered as ``orbbec_pyorbbecsdk``).

Wraps one Orbbec RGB-D camera's ``Pipeline`` behind the
:class:`~hirobot.devices.camera.PollingCameraDriver` contract: the colour stream
and, with ``depth: true``, the depth stream aligned to colour by the camera
(``OBAlignMode.HW_MODE``, or ``SW_MODE`` in the SDK) so both share one pixel grid.

Written against the 1.x Python bindings (`pyorbbecsdk
<https://github.com/orbbec/pyorbbecsdk>`_ on its ``v1`` line, over OrbbecSDK
1.10). Older cameras such as the Dabai DC1 only enumerate there; the 2.x
bindings (``pyorbbecsdk2``) do not list them. Build the 1.x bindings from
source and put them on ``PYTHONPATH``.

Pick the camera with ``serial`` or ``uid``. ``serial`` is the serial number the
device reports once opened; some cameras (the Dabai DC1) leave it empty in the
device list, and then the driver opens the devices one by one to read it, which
can disturb a camera another process is streaming. ``uid`` is the USB port path
the SDK lists (e.g. ``3-1.1.3.4-16``) and selects without opening anything else.

Colour arrives as MJPEG by default and is decoded with OpenCV (``pip install
opencv-python``). Depth arrives as uint16 in device units; ``get_depth_scale()``
(millimetres per unit) converts it to metres, and 0 stays 0 (no reading).
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

import numpy as np

from hirobot.devices.camera import PollingCameraDriver, register_camera
from hirobot.loading.sdk import require_sdk


@register_camera("orbbec_pyorbbecsdk")
class OrbbecCamera(PollingCameraDriver):
    """Frames from one Orbbec camera, with aligned depth for an RGB-D camera.

    Args:
        name: Camera name (the ``images`` / ``depth`` key).
        serial: Device serial number to open (see the module notes).
        uid: The SDK's device uid (USB port path) to open; used before ``serial``.
            With neither, the first device the SDK lists is opened.
        width / height: Colour stream size in pixels.
        fps: Stream frame rate.
        color_format: Colour stream format as the SDK names it: ``MJPG``,
            ``RGB`` or ``YUYV``.
        depth: Stream depth and align it to colour; frames then carry a depth
            image in metres at the colour size.
        align: ``hw`` (on the camera) or ``sw`` (in the SDK).
        timeout_ms: How long one ``wait_for_frames`` blocks.
        poll_hz: Grab attempts per second; at or above ``fps``.
        network_interface: Ignored (``make_robot`` fills it in for every
            hardware camera); these are local USB devices.
    """

    def __init__(
        self,
        name: str,
        serial: Optional[str] = None,
        uid: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        color_format: str = "MJPG",
        depth: bool = False,
        align: str = "hw",
        timeout_ms: int = 1000,
        poll_hz: float = 60.0,
        network_interface: Optional[str] = None,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        self.serial = None if serial is None else str(serial)
        self.uid = None if uid is None else str(uid)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.color_format = str(color_format).upper()
        if self.color_format not in ("MJPG", "RGB", "YUYV"):
            raise ValueError(f"color_format must be MJPG, RGB or YUYV, got {color_format!r}")
        self.depth = bool(depth)
        if align not in ("hw", "sw"):
            raise ValueError(f"align must be 'hw' or 'sw', got {align!r}")
        self.align = align
        self._timeout_ms = int(timeout_ms)
        self._ob: Any = None
        self._cv2: Any = None
        self._pipeline: Any = None
        self._last_stamp: Optional[int] = None

    # ------------------------------------------------------------ opening

    def _find_device(self, ob: Any) -> Any:
        devices = ob.Context().query_devices()
        count = devices.get_count()
        if count == 0:
            raise RuntimeError(f"camera {self.name!r}: no Orbbec device found")
        if self.uid is not None:
            for i in range(count):
                if devices.get_device_uid_by_index(i) == self.uid:
                    return devices.get_device_by_index(i)
            raise RuntimeError(f"camera {self.name!r}: no Orbbec device with uid {self.uid}")
        if self.serial is None:
            return devices.get_device_by_index(0)
        listed = [devices.get_device_serial_number_by_index(i) for i in range(count)]
        if self.serial in listed:
            return devices.get_device_by_index(listed.index(self.serial))
        for i in range(count):
            if listed[i]:
                continue  # its listed serial is known and is not ours
            device = devices.get_device_by_index(i)
            if device.get_device_info().get_serial_number() == self.serial:
                return device
        raise RuntimeError(f"camera {self.name!r}: no Orbbec device with serial {self.serial}")

    def _open(self) -> None:
        require_sdk(__package__, "OrbbecCamera")
        try:
            import pyorbbecsdk as ob
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "OrbbecCamera needs the pyorbbecsdk 1.x bindings "
                "(https://github.com/orbbec/pyorbbecsdk, v1 branch) on PYTHONPATH.") from exc
        if self.color_format != "RGB":
            try:
                import cv2
            except ImportError as exc:  # pragma: no cover - import guard
                raise ImportError("OrbbecCamera needs OpenCV to decode colour; "
                                  "install it with 'pip install opencv-python'.") from exc
            self._cv2 = cv2
        self._ob = ob
        pipeline = ob.Pipeline(self._find_device(ob))
        config = ob.Config()
        fmt = getattr(ob.OBFormat, self.color_format)
        color = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR) \
            .get_video_stream_profile(self.width, self.height, fmt, self.fps)
        config.enable_stream(color)
        if self.depth:
            # Depth size is the camera's default; alignment brings it to the colour grid.
            depth = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR) \
                .get_default_video_stream_profile()
            config.enable_stream(depth)
            config.set_align_mode(ob.OBAlignMode.HW_MODE if self.align == "hw"
                                  else ob.OBAlignMode.SW_MODE)
            try:
                pipeline.enable_frame_sync()
            except Exception:  # noqa: BLE001 - some cameras (Dabai DC1) have no frame sync
                pass
        pipeline.start(config)
        self._pipeline = pipeline

    # ------------------------------------------------------------ frames

    def _decode(self, frame: Any) -> np.ndarray:
        data = np.asanyarray(frame.get_data())
        w, h = frame.get_width(), frame.get_height()
        if self.color_format == "MJPG":
            buf = data.reshape(-1).view(np.uint8)
            bgr = self._cv2.imdecode(buf, self._cv2.IMREAD_COLOR)
            if bgr is None:
                hint = ""
                if buf.size and not (buf != 0xFF).any():
                    # Seen with bindings built against numpy 1.x running under numpy 2.
                    hint = ("; the buffer is all 0xff, which happens when pyorbbecsdk was "
                            f"built against another numpy major version (running {np.__version__})")
                raise RuntimeError(f"camera {self.name!r}: could not decode an MJPEG frame{hint}")
            return np.ascontiguousarray(bgr[:, :, ::-1])
        if self.color_format == "YUYV":
            return self._cv2.cvtColor(data.reshape(h, w, 2), self._cv2.COLOR_YUV2RGB_YUY2)
        return np.ascontiguousarray(data.reshape(h, w, 3))

    def _grab(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        frames = self._pipeline.wait_for_frames(self._timeout_ms)
        if frames is None:
            raise TimeoutError(f"camera {self.name!r}: no frames within {self._timeout_ms} ms; "
                               "is the Orbbec camera connected?")
        color = frames.get_color_frame()
        if color is None:
            return None
        stamp = color.get_timestamp()
        if stamp == self._last_stamp:
            return None  # a frame already handed out
        depth_frame = frames.get_depth_frame() if self.depth else None
        if self.depth and depth_frame is None:
            return None  # wait for a pair
        self._last_stamp = stamp
        image = self._decode(color)
        depth = None
        if depth_frame is not None:
            raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
                depth_frame.get_height(), depth_frame.get_width())
            depth = raw.astype(np.float32) * np.float32(depth_frame.get_depth_scale() / 1000.0)
            if depth.shape != image.shape[:2]:
                # The camera aligns only some colour sizes (a Dabai DC1: 640x480, not 1280x720).
                raise RuntimeError(
                    f"camera {self.name!r}: depth {depth.shape[1]}x{depth.shape[0]} is not aligned "
                    f"to colour {image.shape[1]}x{image.shape[0]}; set align: sw")
        return image, depth

    def _close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001 - already stopped
                pass
            self._pipeline = None
        self._last_stamp = None
