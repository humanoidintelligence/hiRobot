"""``stereolabs_zed`` passes the camera contract against a fake pyzed."""

import numpy as np
import pytest

from hirobot.drivers.camera import ZedCamera
from tests.drivers.camera.stereolabs.zed import fake_sdk
from tests.drivers.contract import check_camera


@pytest.fixture(autouse=True)
def _sdk_found(monkeypatch):
    # The fake has no package metadata; skip the manifest version check.
    monkeypatch.setattr("hirobot.drivers.camera.stereolabs.zed.driver.require_sdk",
                        lambda *a, **k: {})


def test_contract(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_image()
    check_camera("stereolabs_zed", lambda: ZedCamera("wrist", serial=10000001))
    assert bus.params.serial == 10000001
    assert bus.params.camera_resolution == "RESOLUTION.HD720"
    assert bus.params.depth_mode == "DEPTH_MODE.NONE"
    assert bus.closed == 1


def test_bgra_becomes_rgb(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_image(rgb=(10, 150, 200))
    cam = ZedCamera("left")
    cam.start()
    try:
        image, _ = cam.wait_for_frame(2.0)
        assert image.shape == (48, 64, 3) and image.dtype == np.uint8
        assert tuple(image[0, 0]) == (10, 150, 200)
        assert image.flags["C_CONTIGUOUS"]
    finally:
        cam.stop()


def test_depth_in_metres_and_no_reading_is_zero(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_image()
    bus.set_depth(metres=1.5)
    cam = ZedCamera("left", depth=True, depth_mode="neural")
    cam.start()
    try:
        cam.wait_for_frame(2.0)
        _, depth, _ = cam.latest_rgbd()
        assert bus.params.depth_mode == "DEPTH_MODE.NEURAL"
        assert depth.dtype == np.float32 and depth.shape == (48, 64)
        assert float(depth.max()) == pytest.approx(1.5)
        assert depth[0, 0] == 0.0 and depth[0, 1] == 0.0 and depth[0, 2] == 0.0
    finally:
        cam.stop()


def test_calibration_warning_still_opens(monkeypatch):
    # A ZED 2i reported POTENTIAL_CALIBRATION_ISSUE on a cold start but streamed.
    bus = fake_sdk.install(monkeypatch, open_status="ERROR_CODE.POTENTIAL_CALIBRATION_ISSUE")
    bus.set_image()
    cam = ZedCamera("left")
    cam.start()
    try:
        assert cam.wait_for_frame(2.0) is not None
    finally:
        cam.stop()


def test_open_failure_raises(monkeypatch):
    bus = fake_sdk.install(monkeypatch, open_status="ERROR_CODE.CAMERA_NOT_DETECTED")
    with pytest.raises(RuntimeError, match="could not open serial 1"):
        ZedCamera("left", serial=1).start()
    assert bus.closed == 1


def test_unknown_resolution_rejected():
    with pytest.raises(ValueError, match="unknown ZED resolution"):
        ZedCamera("left", resolution="HD999")


def test_resolution_sets_size():
    cam = ZedCamera("left", resolution="vga")
    assert (cam.width, cam.height) == (672, 376)


def test_intrinsics_from_the_sdk(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_image()
    cam = ZedCamera("wrist")
    assert cam.intrinsics() is None  # not started
    cam.start()
    try:
        intr = cam.intrinsics()
        assert (intr.width, intr.height) == (1280, 720)
        assert (intr.fx, intr.fy, intr.cx, intr.cy) == (700.0, 700.0, 640.5, 360.25)
        assert intr.model == "none"  # the image is rectified
    finally:
        cam.stop()


def test_frames_flow_without_intrinsics(monkeypatch, caplog):
    bus = fake_sdk.install(monkeypatch)
    bus.set_image()
    bus.left = None
    cam = ZedCamera("wrist")
    cam.start()
    try:
        assert cam.intrinsics() is None and "could not read the left intrinsics" in caplog.text
        cam.wait_for_frame(2.0)
        assert cam.latest() is not None
    finally:
        cam.stop()
