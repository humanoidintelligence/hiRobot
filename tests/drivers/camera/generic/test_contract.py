"""The stand-in cameras pass the camera contract."""

import numpy as np

from hirobot.drivers.camera import ArrayCamera, VideoFileCamera
from tests.drivers.contract import check_camera


def test_array_contract():
    check_camera("array", lambda: ArrayCamera("c", frames=[np.zeros((8, 8, 3), np.uint8)]))


def test_video_file_contract(tmp_path):
    from tests.drivers.camera.generic.test_video_file import _make_video

    path = _make_video(tmp_path / "v.mp4")  # skips without PyAV
    check_camera("video_file", lambda: VideoFileCamera("c", path, fps=20.0))
