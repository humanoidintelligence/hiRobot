"""``video_file``: frames from a video file (needs PyAV)."""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot.drivers.camera import VideoFileCamera


def _make_video(path, n: int = 6, fps: int = 10) -> str:
    av = pytest.importorskip("av")
    container = av.open(str(path), mode="w")
    stream = container.add_stream("mpeg4", rate=fps)
    stream.width, stream.height, stream.pix_fmt = 32, 32, "yuv420p"
    for i in range(n):
        img = np.full((32, 32, 3), min(255, 40 * i), dtype=np.uint8)
        for packet in stream.encode(av.VideoFrame.from_ndarray(img, format="rgb24")):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()
    return str(path)


def test_video_file_camera_plays_and_loops(tmp_path):
    path = _make_video(tmp_path / "v.mp4", n=4, fps=10)
    cam = VideoFileCamera("v", path, fps=10, loop=True)
    cam.start()
    try:
        first = cam.wait_for_frame(2.0)[0]
        assert first.shape == (32, 32, 3)
        # Four frames at 10 fps take 0.4 s; after a second it has looped.
        deadline = time.time() + 3.0
        while cam.frames < 8 and time.time() < deadline:
            time.sleep(0.05)
        assert cam.frames >= 8 and not cam.ended
    finally:
        cam.stop()


def test_video_file_camera_keeps_last_frame_at_end(tmp_path):
    path = _make_video(tmp_path / "v.mp4", n=3, fps=10)
    cam = VideoFileCamera("v", path, fps=10, loop=False)
    cam.start()
    try:
        cam.wait_for_frame(2.0)
        deadline = time.time() + 3.0
        while not cam.ended and time.time() < deadline:
            time.sleep(0.05)
        assert cam.ended
        last = cam.latest()[0]
        assert int(last.mean()) > 60  # the brightest (last) frame, not the first
    finally:
        cam.stop()


def test_video_file_camera_is_paced(tmp_path):
    path = _make_video(tmp_path / "v.mp4", n=10, fps=10)
    cam = VideoFileCamera("v", path, fps=10, loop=False)
    cam.start()
    try:
        cam.wait_for_frame(2.0)
        time.sleep(0.3)
        assert cam.frames <= 6  # ~0.3 s of a 10 fps video, not all ten frames
    finally:
        cam.stop()
