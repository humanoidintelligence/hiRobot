"""Fake ``pyorbbecsdk`` 1.x: devices, a pipeline, colour (MJPEG / RGB) and depth frames.

Models only what ``OrbbecCamera`` touches: ``Context().query_devices()`` with the
per-index getters, ``Pipeline(device)`` with stream profile lists, ``Config``,
``start`` / ``stop`` / ``wait_for_frames``, and frame sets.
"""

from __future__ import annotations

import sys
import types
from enum import Enum
from types import SimpleNamespace as NS
from typing import List, Optional

import numpy as np


class OBFormat(Enum):
    MJPG = "MJPG"
    RGB = "RGB"
    YUYV = "YUYV"
    Y11 = "Y11"
    Y16 = "Y16"


class OBSensorType(Enum):
    COLOR_SENSOR = "color"
    DEPTH_SENSOR = "depth"


class OBAlignMode(Enum):
    DISABLE = 0
    HW_MODE = 1
    SW_MODE = 2


class Device:
    def __init__(self, uid: str, serial: str, listed_serial: str):
        self.uid, self.serial, self.listed_serial = uid, serial, listed_serial

    def get_device_info(self):
        return NS(get_serial_number=lambda: self.serial)


class Frame:
    def __init__(self, data: np.ndarray, w: int, h: int, fmt: OBFormat, ts: int, scale: float = 1.0):
        self._data, self._w, self._h, self._fmt, self._ts, self._scale = data, w, h, fmt, ts, scale

    def get_data(self):
        return self._data

    def get_width(self):
        return self._w

    def get_height(self):
        return self._h

    def get_format(self):
        return self._fmt

    def get_timestamp(self):
        return self._ts

    def get_depth_scale(self):
        return self._scale


class Bus:
    """The attached devices, what was opened and configured, and the frames handed out."""

    def __init__(self):
        self.devices: List[Device] = [
            Device("3-1.1.1.4-13", "CC1N0000AAAA", ""),
            Device("3-1.1.3.4-16", "CC1N0000BBBB", ""),
        ]
        self.opened: List[str] = []          # uids a Pipeline was built on
        self.info_reads: List[str] = []      # uids whose device info was read
        self.color_profile = None
        self.depth_enabled = False
        self.align = None
        self.started = False
        self.stopped = 0
        self.color: Optional[np.ndarray] = None      # HxWx3 RGB
        self.color_format = OBFormat.MJPG
        self.depth_raw: Optional[np.ndarray] = None  # HxW uint16
        self.depth_scale = 1.0                        # mm per unit
        self.ts = 0
        self.garbage = False                          # colour buffers all 0xff
        self.fps = [(640, 480, 30, OBFormat.MJPG), (1280, 720, 30, OBFormat.MJPG),
                    (640, 480, 30, OBFormat.RGB)]

    def set_color(self, h=480, w=640):
        self.color = np.zeros((h, w, 3), dtype=np.uint8)
        self.color[..., 0] = 200  # red
        self.color[..., 2] = 10

    def set_depth(self, raw=1500, h=480, w=640):
        self.depth_raw = np.full((h, w), raw, dtype=np.uint16)
        self.depth_raw[0, 0] = 0

    def frameset(self):
        if self.color is None:
            return None
        self.ts += 33
        h, w = self.color.shape[:2]
        if self.color_format == OBFormat.MJPG:
            import cv2
            ok, buf = cv2.imencode(".jpg", self.color[:, :, ::-1])
            data = buf.reshape(-1) if not self.garbage else np.full(buf.size, 0xFF, np.uint8)
        else:
            data = self.color.reshape(-1).copy()
        color = Frame(data, w, h, self.color_format, self.ts)
        depth = None
        if self.depth_enabled and self.depth_raw is not None:
            dh, dw = self.depth_raw.shape
            depth = Frame(self.depth_raw.tobytes(), dw, dh, OBFormat.Y16, self.ts, self.depth_scale)
        return NS(get_color_frame=lambda: color, get_depth_frame=lambda: depth)


def install(monkeypatch) -> Bus:
    bus = Bus()

    class DeviceList:
        def get_count(self):
            return len(bus.devices)

        def get_device_uid_by_index(self, i):
            return bus.devices[i].uid

        def get_device_serial_number_by_index(self, i):
            return bus.devices[i].listed_serial

        def get_device_by_index(self, i):
            dev = bus.devices[i]
            orig = dev.get_device_info

            def info():
                bus.info_reads.append(dev.uid)
                return orig()
            dev.get_device_info = info
            return dev

    class Context:
        def query_devices(self):
            return DeviceList()

    class ProfileList:
        def __init__(self, sensor):
            self.sensor = sensor

        def get_video_stream_profile(self, w, h, fmt, fps):
            if (w, h, fps, fmt) not in bus.fps:
                raise RuntimeError(f"no profile {w}x{h}@{fps} {fmt}")
            return ("color", w, h, fps, fmt)

        def get_default_video_stream_profile(self):
            return ("depth", 640, 400, 30, OBFormat.Y11) if self.sensor == OBSensorType.DEPTH_SENSOR \
                else ("color", 640, 480, 30, OBFormat.MJPG)

    class Config:
        def enable_stream(self, profile):
            if profile[0] == "color":
                bus.color_profile = profile
                bus.color_format = profile[4]
            else:
                bus.depth_enabled = True

        def set_align_mode(self, mode):
            bus.align = mode

    class Pipeline:
        def __init__(self, device):
            bus.opened.append(device.uid)

        def get_stream_profile_list(self, sensor):
            return ProfileList(sensor)

        def enable_frame_sync(self):
            raise RuntimeError("Current device does not support frame sync!")

        def start(self, config):
            bus.started = True

        def stop(self):
            bus.stopped += 1

        def wait_for_frames(self, timeout_ms):
            return bus.frameset()

    mod = types.ModuleType("pyorbbecsdk")
    mod.Context, mod.Pipeline, mod.Config = Context, Pipeline, Config
    mod.OBFormat, mod.OBSensorType, mod.OBAlignMode = OBFormat, OBSensorType, OBAlignMode
    monkeypatch.setitem(sys.modules, "pyorbbecsdk", mod)
    return bus
