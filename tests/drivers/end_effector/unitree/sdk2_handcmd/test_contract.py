"""``unitree_handcmd`` (Dex3-1) passes the hand contract against the fake SDK."""

from hirobot import load_embodiment
from hirobot.devices.end_effector import build_end_effectors
from tests.drivers.contract import check_hand
from tests.drivers.end_effector.unitree.sdk2_handcmd.fake_sdk import _Bus, _install_fake_sdk


def test_contract(monkeypatch):
    bus = _Bus()
    _install_fake_sdk(monkeypatch, bus, {"rt/dex3/left/state": [0.1] * 7})
    emb = load_embodiment("unitree_g1_dex3")
    check_hand("unitree_handcmd", lambda ro: build_end_effectors(emb, "eth0", read_only=ro)["left_hand"],
               lambda: sum(len(v) for v in bus.sent.values()))
