"""AgibotCosineCamera: colour decoding, depth pairing and scaling (fake cosine_bus)."""

import time

import numpy as np
import pytest

from hirobot.drivers.camera import AgibotCosineCamera
from tests.drivers.camera.agibot.cosine import fake_sdk

pytest.importorskip("cv2")

HEAD, DEPTH = "/camera/head_color", "/camera/head_depth"


def _camera(**kw):
    return AgibotCosineCamera("head", HEAD, depth_topic=DEPTH, poll_hz=200.0, **kw)


def _wait_seq(cam, n, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cam.frames >= n:
            return
        time.sleep(0.005)
    raise AssertionError(f"only {cam.frames} frames")


def test_colour_is_rgb_and_depth_is_metres(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.color(HEAD, seq=7, source_ns=5_000_000)
    bus.depth(DEPTH, seq=7, source_ns=5_000_000, raw=1669)
    cam = _camera()
    cam.start()
    try:
        cam.wait_for_frame(1.0)
        image, depth, stamp = cam.latest_rgbd()
    finally:
        cam.stop()
    assert image.shape == (48, 64, 3) and image.dtype == np.uint8
    assert abs(int(image[..., 0].mean()) - 200) < 10 and image[..., 2].mean() < 20
    assert depth.shape == (48, 64) and depth.dtype == np.float32
    assert depth[0, 0] == 0.0 and depth[1, 1] == pytest.approx(1.669)
    assert stamp.seq == 1 and cam.unpaired == 0


def test_depth_from_another_capture_is_dropped(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.depth(DEPTH, seq=1, source_ns=0)
    bus.color(HEAD, seq=2, source_ns=33_000_000)  # 33 ms later: the next capture
    cam = _camera()
    cam.start()
    try:
        cam.wait_for_frame(1.0)
        image, depth, _ = cam.latest_rgbd()
    finally:
        cam.stop()
    assert image is not None and depth is None and cam.unpaired == 1


def test_colour_waits_for_late_depth(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.color(HEAD, seq=1, source_ns=0)
    cam = _camera(pair_wait_s=1.0)
    cam.start()
    try:
        time.sleep(0.05)
        assert cam.latest() is None  # held back, waiting for its depth
        bus.depth(DEPTH, seq=1, source_ns=0)
        cam.wait_for_frame(1.0)
        assert cam.latest_rgbd()[1] is not None and cam.unpaired == 0
    finally:
        cam.stop()


def test_nearest_depth_is_picked(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.depth(DEPTH, seq=1, source_ns=0, raw=1000)
    bus.depth(DEPTH, seq=2, source_ns=33_000_000, raw=2000)
    bus.color(HEAD, seq=1, source_ns=0)
    cam = _camera()
    cam.start()
    try:
        cam.wait_for_frame(1.0)
        assert cam.latest_rgbd()[1][1, 1] == pytest.approx(1.0)
    finally:
        cam.stop()


def test_a_repeated_packet_is_not_a_new_frame(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.color(HEAD, seq=1, source_ns=0)
    cam = AgibotCosineCamera("head", HEAD, poll_hz=200.0)
    cam.start()
    try:
        _wait_seq(cam, 1)
        time.sleep(0.05)
        assert cam.frames == 1 and cam.errors == 0
        bus.color(HEAD, seq=2, source_ns=33_000_000)
        _wait_seq(cam, 2)
    finally:
        cam.stop()


def test_colour_only_camera_gives_no_depth(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.color(HEAD, seq=1, source_ns=0)
    cam = AgibotCosineCamera("hand_left", HEAD, poll_hz=200.0)
    cam.start()
    try:
        cam.wait_for_frame(1.0)
        assert cam.latest_rgbd()[1] is None
    finally:
        cam.stop()
    assert bus.subscribed == [HEAD]


def test_missing_vendor_environment_is_named(monkeypatch):
    fake_sdk.install(monkeypatch, env=False)
    with pytest.raises(RuntimeError, match="LOCATOR_IP.*env.sh"):
        _camera().start()
