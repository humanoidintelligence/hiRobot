"""``unitree_image_server``: colour order, header, crop, errors, and the H1 config."""

import numpy as np
import pytest

pytest.importorskip("cv2")
pytest.importorskip("zmq")

from hirobot import RobotAssembly, make_robot  # noqa: E402
from hirobot.drivers.camera import UnitreeImageServerCamera  # noqa: E402
from tests.drivers.camera.unitree.image_server.fake_sdk import FakeImageServer  # noqa: E402


def _first_frame(server, **kw):
    cam = UnitreeImageServerCamera("head", address=server.address, **kw)
    cam.start()
    try:
        return cam, cam.wait_for_frame(3.0)[0]
    finally:
        cam.stop()


def test_frames_are_rgb():
    server = FakeImageServer()
    try:
        _, image = _first_frame(server)
    finally:
        server.close()
    assert image.shape == (48, 64, 3) and image.dtype == np.uint8
    left, right = image[:, :24].astype(int), image[:, 40:].astype(int)
    assert np.abs(left[..., 0] - 200).max() < 15 and left[..., 2].max() < 15   # red
    assert np.abs(right[..., 2] - 200).max() < 15 and right[..., 0].max() < 15  # blue


def test_header_is_stripped_and_kept():
    server = FakeImageServer(header=True)
    try:
        cam, image = _first_frame(server, header=True)
    finally:
        server.close()
    assert image.shape == (48, 64, 3)
    assert cam.remote_frame_id is not None and cam.remote_stamp > 0


def test_crop_takes_one_camera_out_of_a_concatenated_frame():
    server = FakeImageServer()
    try:
        _, image = _first_frame(server, crop=[32, 0, 32, 48])
    finally:
        server.close()
    assert image.shape == (48, 32, 3)
    assert image[..., 2].mean() > 150 and image[..., 0].mean() < 30  # the blue half


def test_crop_outside_the_frame_is_an_error():
    server = FakeImageServer()
    try:
        cam = UnitreeImageServerCamera("head", address=server.address, crop=[40, 0, 64, 48])
        cam.start()
        try:
            with pytest.raises(TimeoutError, match="outside"):
                cam.wait_for_frame(0.5)
        finally:
            cam.stop()
    finally:
        server.close()


def test_bad_crop_spec():
    with pytest.raises(ValueError, match="crop"):
        UnitreeImageServerCamera("head", address="tcp://127.0.0.1:1", crop=[0, 0, 10])


def test_not_a_jpeg():
    server = FakeImageServer(payload=b"not a jpeg")
    try:
        cam = UnitreeImageServerCamera("head", address=server.address)
        cam.start()
        try:
            with pytest.raises(TimeoutError, match="not a decodable JPEG"):
                cam.wait_for_frame(0.5)
        finally:
            cam.stop()
    finally:
        server.close()


def test_no_server_names_the_cause():
    cam = UnitreeImageServerCamera("head", address="tcp://127.0.0.1:1", timeout_s=0.1)
    cam.start()
    try:
        with pytest.raises(TimeoutError, match="is the image server running"):
            cam.wait_for_frame(0.5)
    finally:
        cam.stop()


@pytest.mark.parametrize("config", ["unitree_h1", "unitree_h1_inspire"])
def test_h1_configs_attach_the_head_camera_on_hardware(config):
    robot = make_robot(config, body_overrides={"driver": "unitree_sdk2", "msg_family": "go",
                                                "network_interface": "eth9"})
    assert isinstance(robot, RobotAssembly)
    cam = robot.cameras["head"]
    assert isinstance(cam, UnitreeImageServerCamera)
    assert cam.address.startswith("tcp://") and cam.address.endswith(":5555")


def test_h1_config_has_no_camera_source_in_sim():
    robot = make_robot("unitree_h1")
    assert not getattr(robot, "cameras", {})
