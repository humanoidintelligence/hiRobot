"""``agibot_cosine`` passes the camera contract against a fake cosine_bus."""

import pytest

from hirobot.drivers.camera import AgibotCosineCamera
from tests.drivers.camera.agibot.cosine import fake_sdk
from tests.drivers.contract import check_camera

pytest.importorskip("cv2")


def test_contract(monkeypatch):
    bus = fake_sdk.install(monkeypatch)
    bus.color("/camera/head_color", seq=1, source_ns=1_000)
    check_camera("agibot_cosine", lambda: AgibotCosineCamera("head", "/camera/head_color"))
    assert bus.subscribed == ["/camera/head_color"]
