"""Fake ``pyzed.sl``: a camera that hands out BGRA images and depth a test sets.

Only the surface ``ZedCamera`` touches is modelled: ``InitParameters``,
``RuntimeParameters``, ``Camera`` (``open`` / ``is_opened`` / ``grab`` /
``retrieve_image`` / ``retrieve_measure`` / ``get_timestamp`` / ``close``),
``Mat`` and the enum holders.
"""

from __future__ import annotations

import sys
import types
from typing import Optional

import numpy as np


class _Enum:
    """A stand-in for an SDK enum: any attribute is a unique token."""

    def __init__(self, prefix: str):
        self._prefix = prefix

    def __getattr__(self, name: str) -> str:
        if name.startswith("__"):
            raise AttributeError(name)
        return f"{self._prefix}.{name}"


class Bus:
    """Shared state: what the fake camera hands out, and how it was opened."""

    def __init__(self, open_status: str = "ERROR_CODE.SUCCESS"):
        self.open_status = open_status
        self.params = None
        self.opened = False
        self.closed = 0
        self.stamp = 0
        self.repeat_stamp = False
        self._bgra: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None

    def set_image(self, rgb=(10, 150, 200), size=(64, 48)) -> None:
        w, h = size
        img = np.zeros((h, w, 4), np.uint8)
        img[..., 0], img[..., 1], img[..., 2] = rgb[2], rgb[1], rgb[0]  # BGRA
        img[..., 3] = 255
        self._bgra = img

    def set_depth(self, metres: float = 1.5, size=(64, 48)) -> None:
        w, h = size
        d = np.full((h, w), metres, np.float32)
        d[0, 0] = np.nan      # no reading
        d[0, 1] = np.inf      # too far
        d[0, 2] = -np.inf     # too near
        self._depth = d


def install(monkeypatch, open_status: str = "ERROR_CODE.SUCCESS") -> Bus:
    bus = Bus(open_status)
    ERROR_CODE = _Enum("ERROR_CODE")

    class InitParameters:  # the SDK's name
        def __init__(self):
            self.serial = None

        def set_from_serial_number(self, serial):
            self.serial = serial

    class RuntimeParameters:  # the SDK's name
        pass

    class Mat:  # the SDK's name
        def __init__(self):
            self._data = None

        def get_data(self):
            return self._data

    class _Timestamp:
        def __init__(self, ns):
            self._ns = ns

        def get_nanoseconds(self):
            return self._ns

    class Camera:  # the SDK's name
        def open(self, params):
            bus.params = params
            bus.opened = bus.open_status in ("ERROR_CODE.SUCCESS", "ERROR_CODE.POTENTIAL_CALIBRATION_ISSUE")
            return bus.open_status

        def is_opened(self):
            return bus.opened

        def grab(self, runtime):
            if not bus.repeat_stamp:
                bus.stamp += 33_000_000
            return ERROR_CODE.SUCCESS

        def get_timestamp(self, ref):
            return _Timestamp(bus.stamp)

        def retrieve_image(self, mat, view):
            assert view == "VIEW.LEFT"
            mat._data = bus._bgra

        def retrieve_measure(self, mat, measure):
            assert measure == "MEASURE.DEPTH"
            mat._data = bus._depth

        def close(self):
            bus.opened = False
            bus.closed += 1

    sl = types.ModuleType("pyzed.sl")
    sl.InitParameters = InitParameters
    sl.RuntimeParameters = RuntimeParameters
    sl.Mat = Mat
    sl.Camera = Camera
    sl.ERROR_CODE = ERROR_CODE
    for enum in ("RESOLUTION", "DEPTH_MODE", "UNIT", "VIEW", "MEASURE", "TIME_REFERENCE"):
        setattr(sl, enum, _Enum(enum))
    pkg = types.ModuleType("pyzed")
    pkg.sl = sl
    monkeypatch.setitem(sys.modules, "pyzed", pkg)
    monkeypatch.setitem(sys.modules, "pyzed.sl", sl)
    return bus
