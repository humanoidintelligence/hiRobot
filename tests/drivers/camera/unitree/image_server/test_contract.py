"""``unitree_image_server`` passes the camera contract against a local ZeroMQ publisher."""

import pytest

pytest.importorskip("cv2")
pytest.importorskip("zmq")

from hirobot.drivers.camera import UnitreeImageServerCamera  # noqa: E402
from tests.drivers.camera.unitree.image_server.fake_sdk import FakeImageServer  # noqa: E402
from tests.drivers.contract import check_camera  # noqa: E402


def test_contract():
    server = FakeImageServer()
    try:
        check_camera("unitree_image_server",
                     lambda: UnitreeImageServerCamera("head", address=server.address))
    finally:
        server.close()
    assert server.sent > 0
