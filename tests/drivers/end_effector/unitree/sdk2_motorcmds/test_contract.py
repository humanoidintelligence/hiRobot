"""``unitree_motorcmds`` (H1 Inspire via h1_inspire_service) passes the hand contract."""

from hirobot import load_embodiment
from hirobot.end_effector import build_end_effectors
from tests.drivers.contract import check_hand
from tests.drivers.end_effector.unitree.sdk2_motorcmds.fake_sdk import _install, _MotorStates


def test_contract(monkeypatch):
    bus = _install(monkeypatch, _MotorStates([0.5] * 12))
    emb = load_embodiment("unitree_h1_inspire")
    check_hand("unitree_motorcmds", lambda ro: build_end_effectors(emb, "eth0", read_only=ro)["left_hand"],
               lambda: len(bus.sent))
