"""``orbbec_pyorbbecsdk`` passes the camera contract against a fake pyorbbecsdk 1.x."""

import numpy as np
import pytest

from hirobot.drivers.camera import OrbbecCamera
from tests.drivers.camera.orbbec.pyorbbecsdk import fake_sdk
from tests.drivers.contract import check_camera

pytest.importorskip("cv2")


@pytest.fixture(autouse=True)
def _sdk_found(monkeypatch):
    # The fake has no package metadata; skip the manifest version check.
    monkeypatch.setattr("hirobot.drivers.camera.orbbec.pyorbbecsdk.driver.require_sdk",
                        lambda *a, **k: {})


def test_contract(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    check_camera("orbbec_pyorbbecsdk", lambda: OrbbecCamera("scene"))
    assert bus.opened == ["3-1.1.1.4-13"] and bus.started and bus.stopped == 1


def test_mjpeg_decoded_to_rgb_and_depth_in_metres(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    bus.set_depth(raw=1500)
    cam = OrbbecCamera("scene", uid="3-1.1.3.4-16", depth=True)
    cam.start()
    try:
        cam.wait_for_frame(2.0)
        image, depth, _ = cam.latest_rgbd()
        assert image.shape == (480, 640, 3) and image.dtype == np.uint8
        assert abs(int(image[100, 100, 0]) - 200) < 8 and image[100, 100, 2] < 30  # red stays red
        assert depth.dtype == np.float32 and depth.shape == (480, 640)
        assert depth[10, 10] == pytest.approx(1.5) and depth[0, 0] == 0.0
    finally:
        cam.stop()
    assert bus.opened == ["3-1.1.3.4-16"]
    assert bus.align == fake_sdk.OBAlignMode.HW_MODE and bus.depth_enabled


def test_colour_profile_and_rgb(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color(h=480, w=640)
    bus.set_depth()
    cam = OrbbecCamera("scene", color_format="RGB", align="sw", depth=True)
    cam.start()
    try:
        image, _ = cam.wait_for_frame(2.0)
        assert image[0, 0, 0] == 200
    finally:
        cam.stop()
    assert bus.color_profile == ("color", 640, 480, 30, fake_sdk.OBFormat.RGB)
    assert bus.align == fake_sdk.OBAlignMode.SW_MODE


def test_all_ff_buffer_names_numpy_mismatch(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    bus.garbage = True
    cam = OrbbecCamera("scene")
    cam.start()
    try:
        with pytest.raises(TimeoutError, match="numpy major version"):
            cam.wait_for_frame(0.5)
    finally:
        cam.stop()


def test_unaligned_depth_raises(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color(h=720, w=1280)
    bus.set_depth(h=400, w=640)
    cam = OrbbecCamera("scene", width=1280, height=720, depth=True)
    cam.start()
    try:
        with pytest.raises(TimeoutError, match="align: sw"):
            cam.wait_for_frame(0.5)
    finally:
        cam.stop()


def test_serial_listed_empty_reads_device_info(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    cam = OrbbecCamera("scene", serial="CC1N0000BBBB")
    cam.start()
    cam.stop()
    assert bus.opened == ["3-1.1.3.4-16"]
    assert bus.info_reads == ["3-1.1.1.4-13", "3-1.1.3.4-16"]


def test_serial_listed_skips_others(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    for d in bus.devices:
        d.listed_serial = d.serial
    bus.set_color()
    cam = OrbbecCamera("scene", serial="CC1N0000BBBB")
    cam.start()
    cam.stop()
    assert bus.opened == ["3-1.1.3.4-16"] and bus.info_reads == []


def test_unknown_device_raises(monkeypatch):
    fake_sdk.install(monkeypatch)
    with pytest.raises(RuntimeError, match="uid 9-9"):
        OrbbecCamera("scene", uid="9-9").start()
    with pytest.raises(RuntimeError, match="serial NOPE"):
        OrbbecCamera("scene", serial="NOPE").start()


def test_missing_profile_raises(monkeypatch):
    fake_sdk.install(monkeypatch)
    with pytest.raises(RuntimeError, match="no profile"):
        OrbbecCamera("scene", width=1920, height=1080).start()


def test_bad_arguments():
    with pytest.raises(ValueError, match="color_format"):
        OrbbecCamera("scene", color_format="H264")
    with pytest.raises(ValueError, match="align"):
        OrbbecCamera("scene", align="gpu")
