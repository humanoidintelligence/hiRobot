"""``array``: frames from memory."""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot.devices.camera import get_camera
from hirobot.drivers.camera import ArrayCamera, VideoFileCamera


def _frames(n: int):
    return [np.full((4, 4, 3), i, dtype=np.uint8) for i in range(n)]


def test_array_camera_is_registered_and_simulated():
    assert get_camera("array") is ArrayCamera
    assert get_camera("video_file") is VideoFileCamera
    assert ArrayCamera.simulated and VideoFileCamera.simulated


def test_array_camera_has_no_frame_before_start():
    cam = ArrayCamera("c", _frames(2))
    assert cam.latest() is None
    cam.start()
    assert cam.latest() is not None
    cam.stop()
    assert cam.latest() is None


def test_array_camera_advance_on_read_steps_and_wraps():
    cam = ArrayCamera("c", _frames(3), fps=10, advance_on_read=True)
    cam.start()
    values = [int(cam.latest()[0][0, 0, 0]) for _ in range(5)]
    assert values == [0, 1, 2, 0, 1]
    stamps = [cam.latest()[1] for _ in range(2)]
    assert stamps[1].t - stamps[0].t == pytest.approx(0.1)
    assert stamps[1].seq == stamps[0].seq + 1


def test_array_camera_follows_the_clock(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    cam = ArrayCamera("c", _frames(4), fps=2)
    cam.start()
    assert int(cam.latest()[0][0, 0, 0]) == 0
    now[0] += 0.6  # frame 1 is due at 0.5 s
    image, stamp = cam.latest()
    assert int(image[0, 0, 0]) == 1 and stamp.t == pytest.approx(1000.5) and stamp.seq == 2
    now[0] += 2.0  # 2.6 s -> frame 5 -> wraps to 1
    assert int(cam.latest()[0][0, 0, 0]) == 1


def test_array_camera_default_frame_and_validation():
    cam = ArrayCamera("c")
    cam.start()
    assert cam.latest()[0].shape == (64, 64, 3)
    with pytest.raises(ValueError):
        ArrayCamera("c", [])
    with pytest.raises(ValueError):
        ArrayCamera("c", fps=0)
