"""``agibot_gdk`` passes the body contract against a fake cosine_bus."""

from hirobot import get_body, load_embodiment
from tests.drivers.body.agibot.gdk import fake_sdk
from tests.drivers.contract import check_robot


def test_contract(monkeypatch):
    gdk = fake_sdk.install(monkeypatch)
    emb = load_embodiment("agibot_g1")
    check_robot("agibot_gdk", lambda: get_body("agibot_gdk")(emb),
                lambda: len(gdk.joint_commands()))
    # disconnect sends MotionStop
    assert gdk.published[-1][0] == "/wbc/motion_stop" and gdk.mode == 0
