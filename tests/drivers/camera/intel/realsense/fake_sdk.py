"""Fake ``pyrealsense2``: a pipeline that hands out colour/depth frames a test sets.

Only the surface ``RealSenseCamera`` touches is modelled: ``config`` with
``enable_device`` / ``enable_stream``, ``pipeline`` with ``start`` / ``stop`` /
``wait_for_frames``, ``align``, the ``stream`` / ``format`` enum holders, and
composite frames with ``get_color_frame`` / ``get_depth_frame``.
"""

from __future__ import annotations

import sys
import types
from typing import Optional

import numpy as np


class _Enum:
    """A stand-in for rs.stream / rs.format: any attribute is a unique token."""

    def __init__(self, prefix: str):
        self._prefix = prefix

    def __getattr__(self, name: str) -> str:
        return f"{self._prefix}.{name}"


class _VideoFrame:
    def __init__(self, data: np.ndarray, number: int):
        self._data = data
        self._number = number

    def __bool__(self) -> bool:
        return self._data is not None

    def get_data(self):
        return self._data

    def get_frame_number(self) -> int:
        return self._number


class _Composite:
    """What wait_for_frames / align.process return."""

    def __init__(self, color: _VideoFrame, depth: Optional[_VideoFrame]):
        self._color = color
        self._depth = depth

    def get_color_frame(self) -> _VideoFrame:
        return self._color

    def get_depth_frame(self):
        return self._depth if self._depth is not None else _VideoFrame(None, -1)


class Bus:
    """Shared state: the frames the fake pipeline hands out, and what was configured."""

    def __init__(self, depth_scale: float = 0.001):
        self.depth_scale = depth_scale
        self.enabled_device: Optional[str] = None
        self.enabled_streams: list = []
        self.started = False
        self.stopped = False
        self._next_number = 1
        self._color: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None

    def set_color(self, rgb=(10, 150, 200), size=(64, 48)) -> None:
        w, h = size
        img = np.zeros((h, w, 3), np.uint8)
        img[:] = rgb
        self._color = img

    def set_depth(self, raw: int = 1500, size=(64, 48)) -> None:
        w, h = size
        z = np.full((h, w), raw, np.uint16)
        z[0, 0] = 0  # one pixel without a reading
        self._depth = z

    def _frames(self) -> _Composite:
        number = self._next_number
        self._next_number += 1
        color = _VideoFrame(self._color, number)
        depth = None if self._depth is None else _VideoFrame(self._depth, number)
        return _Composite(color, depth)


def install(monkeypatch, depth_scale: float = 0.001) -> Bus:
    bus = Bus(depth_scale=depth_scale)

    class config:  # the SDK's name
        def __init__(self):
            pass

        def enable_device(self, serial):
            bus.enabled_device = serial

        def enable_stream(self, stream, width, height, fmt, fps):
            bus.enabled_streams.append((stream, width, height, fmt, fps))

    class _DepthSensor:
        def get_depth_scale(self):
            return bus.depth_scale

    class _Device:
        def first_depth_sensor(self):
            return _DepthSensor()

    class _Profile:
        def get_device(self):
            return _Device()

    class pipeline:  # the SDK's name
        def __init__(self):
            pass

        def start(self, config):
            bus.started = True
            return _Profile()

        def stop(self):
            if not bus.started or bus.stopped:
                raise RuntimeError("pipeline not started")
            bus.stopped = True

        def wait_for_frames(self, timeout_ms):
            return bus._frames()

    class align:  # the SDK's name
        def __init__(self, stream):
            self.stream = stream

        def process(self, frames):
            return frames

    mod = types.ModuleType("pyrealsense2")
    mod.config = config
    mod.pipeline = pipeline
    mod.align = align
    mod.stream = _Enum("stream")
    mod.format = _Enum("format")
    monkeypatch.setitem(sys.modules, "pyrealsense2", mod)
    return bus

