"""``inspire_sdkpy`` passes the hand contract against a fake inspire_sdkpy bridge."""

from hirobot import load_embodiment
from hirobot.devices.end_effector import build_end_effectors
from tests.drivers.contract import check_hand
from tests.drivers.end_effector.inspire.sdkpy.fake_sdk import _HandState, _install_sdkpy


def test_contract(monkeypatch):
    sent, _ = _install_sdkpy(monkeypatch, {"rt/inspire_hand/state/l": _HandState([500] * 6)})
    emb = load_embodiment("unitree_h1_inspire")
    for ee in emb.end_effectors:
        ee.driver = {"type": "inspire_sdkpy", "side": ee.name.split("_")[0]}
    check_hand("inspire_sdkpy", lambda ro: build_end_effectors(emb, "eth0", read_only=ro)["left_hand"],
               lambda: len(sent))
