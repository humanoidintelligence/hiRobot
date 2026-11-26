"""Stereolabs ZED camera driver over ``pyzed`` (registered as ``stereolabs_zed``).

Wraps one ZED (ZED 2 / 2i, ZED Mini, ZED X ...) behind the
:class:`~hirobot.devices.camera.PollingCameraDriver` contract. One driver drives
one physical camera, selected by ``serial`` so a fixed camera keeps its role
when several are plugged in. The image is the **left** sensor, rectified, as
HxWx3 uint8 RGB. With ``depth: true`` the SDK's depth map (computed on the GPU,
already registered to the left image) comes along in metres; pixels without a
reading (NaN / +-inf in the SDK) become 0, as for the other RGB-D drivers.

``pyzed`` ships with the ZED SDK (it needs CUDA and an NVIDIA GPU); it is
imported lazily in ``_open``. Run on two ZED 2i and a ZED Mini at once (ZED SDK
5.1.2, HD720 @ 30); see ``docs/setup/franka.md``.
"""

from __future__ import annotations

import logging
from typing import Any, Optional, Tuple

import numpy as np

from hirobot.core.embodiment import CameraIntrinsics
from hirobot.devices.camera import PollingCameraDriver, register_camera
from hirobot.loading.sdk import require_sdk

logger = logging.getLogger(__name__)

# (width, height) of the SDK's named resolutions, per side.
RESOLUTIONS = {
    "HD2K": (2208, 1242),
    "HD1080": (1920, 1080),
    "HD1200": (1920, 1200),
    "HD720": (1280, 720),
    "SVGA": (960, 600),
    "VGA": (672, 376),
}


@register_camera("stereolabs_zed")
class ZedCamera(PollingCameraDriver):
    """Left-eye frames from one Stereolabs ZED, with depth in metres when asked.

    Args:
        name: Camera name (the ``images`` / ``depth`` key).
        serial: Camera serial number. ``None`` opens the first ZED the SDK finds.
        resolution: SDK resolution name (``HD720``, ``HD1080``, ``HD2K``,
            ``VGA``, ...). What each model supports is in the ZED docs; the
            SDK refuses unsupported ones at open.
        fps: Frame rate (15, 30, 60, ... per resolution).
        depth: Compute depth and return it with each frame.
        depth_mode: SDK depth mode name used when ``depth`` is true
            (``NEURAL``, ``NEURAL_LIGHT``, ``PERFORMANCE``, ...).
        min_depth_m: Closest depth the SDK reports (``depth_minimum_distance``).
        poll_hz: Grab attempts per second; at or above ``fps``.
        network_interface: Ignored (``make_robot`` fills it in for every
            hardware camera); a ZED here is a local USB device.
    """

    def __init__(
        self,
        name: str,
        serial: Optional[int] = None,
        resolution: str = "HD720",
        fps: int = 30,
        depth: bool = False,
        depth_mode: str = "NEURAL",
        min_depth_m: float = 0.1,
        poll_hz: float = 60.0,
        network_interface: Optional[str] = None,
    ) -> None:
        super().__init__(name, poll_hz=poll_hz)
        self.serial = None if serial is None else int(serial)
        self.resolution = str(resolution).upper()
        if self.resolution not in RESOLUTIONS:
            raise ValueError(f"camera {name!r}: unknown ZED resolution {resolution!r}; "
                             f"one of {sorted(RESOLUTIONS)}")
        self.width, self.height = RESOLUTIONS[self.resolution]
        self.fps = int(fps)
        self.depth = bool(depth)
        self.depth_mode = str(depth_mode).upper()
        self.min_depth_m = float(min_depth_m)
        self._sl: Any = None
        self._cam: Any = None
        self._runtime: Any = None
        self._image: Any = None
        self._depth_mat: Any = None
        self._last_stamp: Optional[int] = None
        self._intrinsics: Optional[CameraIntrinsics] = None

    def _open(self) -> None:
        require_sdk(__package__, "ZedCamera")
        try:
            import pyzed.sl as sl
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "ZedCamera needs pyzed, which comes with the ZED SDK: install the SDK, "
                "then run its get_python_api.py.") from exc
        self._sl = sl
        params = sl.InitParameters()
        if self.serial is not None:
            params.set_from_serial_number(self.serial)
        params.camera_resolution = getattr(sl.RESOLUTION, self.resolution)
        params.camera_fps = self.fps
        params.coordinate_units = sl.UNIT.METER
        params.depth_mode = getattr(sl.DEPTH_MODE, self.depth_mode) if self.depth else sl.DEPTH_MODE.NONE
        params.depth_minimum_distance = self.min_depth_m
        cam = sl.Camera()
        status = cam.open(params)
        # The SDK opens the camera but warns when its self-check doubts the factory
        # calibration (seen on a ZED 2i on a cold start); the camera still streams.
        warning = getattr(sl.ERROR_CODE, "POTENTIAL_CALIBRATION_ISSUE", None)
        if status != sl.ERROR_CODE.SUCCESS and not (status == warning and cam.is_opened()):
            cam.close()
            which = f"serial {self.serial}" if self.serial is not None else "the first ZED"
            raise RuntimeError(f"camera {self.name!r}: could not open {which}: {status}")
        self._cam = cam
        self._intrinsics = self._read_intrinsics(cam)
        self._runtime = sl.RuntimeParameters()
        self._image = sl.Mat()
        self._depth_mat = sl.Mat()

    def _read_intrinsics(self, cam: Any) -> Optional[CameraIntrinsics]:
        # The image is the rectified left view, so the left camera's pinhole has no distortion.
        try:
            info = cam.get_camera_information()
            conf = getattr(info, "camera_configuration", info)  # SDK 4 / SDK 3
            left = conf.calibration_parameters.left_cam
            size = left.image_size
            return CameraIntrinsics(width=int(size.width), height=int(size.height),
                                    fx=float(left.fx), fy=float(left.fy),
                                    cx=float(left.cx), cy=float(left.cy))
        except Exception as exc:  # noqa: BLE001 - frames still flow without intrinsics
            logger.warning("camera %s: could not read the left intrinsics: %s", self.name, exc)
            return None

    def intrinsics(self) -> Optional[CameraIntrinsics]:
        """The rectified left camera's intrinsics from the SDK, read on start (no distortion)."""
        return self._intrinsics

    def _grab(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        sl = self._sl
        status = self._cam.grab(self._runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"camera {self.name!r}: grab failed: {status}")
        # Skip a frame already handed out (same image timestamp).
        stamp = self._cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        if stamp == self._last_stamp:
            return None
        self._last_stamp = stamp
        self._cam.retrieve_image(self._image, sl.VIEW.LEFT)
        bgra = self._image.get_data()  # HxWx4 uint8 BGRA, a view into SDK memory
        image = np.ascontiguousarray(bgra[:, :, 2::-1])  # -> RGB copy
        depth = None
        if self.depth:
            self._cam.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH)
            depth = np.array(self._depth_mat.get_data(), dtype=np.float32, copy=True)
            depth[~np.isfinite(depth)] = 0.0  # NaN = no reading, +-inf = too near / far
        return image, depth

    def _close(self) -> None:
        if self._cam is not None:
            self._cam.close()
            self._cam = None
        self._last_stamp = None
