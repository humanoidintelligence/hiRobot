"""``intel_realsense`` passes the camera contract against a fake pyrealsense2."""

from hirobot.drivers.camera import RealSenseCamera
from tests.drivers.camera.intel.realsense import fake_sdk
from tests.drivers.contract import check_camera


def test_contract(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    check_camera("intel_realsense", lambda: RealSenseCamera("wrist", serial="123456789"))
    assert bus.enabled_device == "123456789"
    assert bus.started


def test_rgbd_depth_in_metres(monkeypatch):
    bus = fake_sdk.install(monkeypatch, depth_scale=0.001)
    bus.set_color()
    bus.set_depth(raw=1500)
    cam = RealSenseCamera("wrist", serial="123456789", depth=True)
    cam.start()
    try:
        cam.wait_for_frame(2.0)
        image, depth, _ = cam.latest_rgbd()
        assert depth is not None
        assert depth.dtype.kind == "f"
        assert abs(float(depth.max()) - 1.5) < 1e-6  # 1500 units * 0.001 m
        assert float(depth[0, 0]) == 0.0           # no-reading pixel stays 0
        assert image.shape[2] == 3
    finally:
        cam.stop()


def test_depth_size_separate_from_colour(monkeypatch):
    # An L515 on USB 2 offers depth only at 320x240 next to 640x480 colour.
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    bus.set_depth()
    cam = RealSenseCamera("scene", depth=True, depth_width=320, depth_height=240)
    cam.start()
    try:
        sizes = {s: (w, h) for s, w, h, _, _ in bus.enabled_streams}
        assert sizes["stream.color"] == (640, 480)
        assert sizes["stream.depth"] == (320, 240)
    finally:
        cam.stop()


def test_intrinsics_from_the_sdk(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    cam = RealSenseCamera("wrist", width=640, height=480)
    assert cam.intrinsics() is None  # not started
    cam.start()
    try:
        intr = cam.intrinsics()
        assert (intr.width, intr.height) == (640, 480)
        assert (intr.fx, intr.fy, intr.cx, intr.cy) == (615.0, 616.0, 321.5, 238.25)
        assert intr.model == "none" and intr.coeffs == ()  # all-zero coefficients
    finally:
        cam.stop()


def test_intrinsics_keep_distortion(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    bus.coeffs = [0.1, -0.2, 0.001, 0.002, 0.03]
    cam = RealSenseCamera("wrist")
    cam.start()
    try:
        intr = cam.intrinsics()
        assert intr.model == "inverse_brown_conrady" and intr.coeffs == (0.1, -0.2, 0.001, 0.002, 0.03)
    finally:
        cam.stop()


def test_frames_flow_without_intrinsics(monkeypatch, caplog):
    bus = fake_sdk.install(monkeypatch)
    bus.set_color()
    bus.intrinsics = None
    cam = RealSenseCamera("wrist")
    cam.start()
    try:
        assert cam.intrinsics() is None and "could not read the colour intrinsics" in caplog.text
        cam.wait_for_frame(2.0)
        assert cam.latest() is not None
    finally:
        cam.stop()
