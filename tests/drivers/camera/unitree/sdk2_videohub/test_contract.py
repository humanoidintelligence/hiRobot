"""``unitree_videohub`` passes the camera contract against a fake videohub service."""

import numpy as np
import pytest

from hirobot.drivers.camera import UnitreeVideohubCamera
from tests.drivers.camera.unitree.sdk2_videohub import fake_sdk
from tests.drivers.contract import check_camera

pytest.importorskip("cv2")


def test_contract(monkeypatch):
    hub = fake_sdk.install(monkeypatch)
    check_camera("unitree_videohub",
                 lambda: UnitreeVideohubCamera("head", network_interface="eth9"))
    assert hub.dds == [(0, "eth9")] and hub.calls > 0


def test_frames_are_rgb(monkeypatch):
    fake_sdk.install(monkeypatch)
    cam = UnitreeVideohubCamera("head", poll_hz=100.0)
    cam.start()
    try:
        image, _ = cam.wait_for_frame(2.0)
    finally:
        cam.stop()
    assert image.shape == (48, 64, 3)
    assert np.abs(image[..., 0].astype(int) - 200).max() < 10 and image[..., 2].max() < 10


def test_no_service_is_named_in_the_error(monkeypatch):
    fake_sdk.install(monkeypatch, code=3102)
    cam = UnitreeVideohubCamera("head", poll_hz=100.0)
    cam.start()
    try:
        with pytest.raises(TimeoutError, match="videohub service is not running"):
            cam.wait_for_frame(0.3)
    finally:
        cam.stop()
