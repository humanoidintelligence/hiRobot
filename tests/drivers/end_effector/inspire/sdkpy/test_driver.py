"""``inspire_sdkpy``: Inspire hands through the inspire_sdkpy DDS bridge, one topic pair
per hand."""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import load_embodiment
from hirobot.drivers.end_effector import InspireSdkpyHand
from hirobot.end_effector import build_end_effectors
from tests.drivers.end_effector.inspire.sdkpy.fake_sdk import _HandState, _install_sdkpy

CFG = "unitree_h1_inspire"


def _sdkpy_hands(read_only=False):
    emb = load_embodiment(CFG)
    for ee in emb.end_effectors:
        ee.driver = {"type": "inspire_sdkpy", "side": ee.name.split("_")[0]}
    hands = build_end_effectors(emb, "eth0", read_only=read_only)
    assert all(isinstance(h, InspireSdkpyHand) for h in hands.values())
    return hands


def test_sdkpy_topics_per_hand():
    hands = _sdkpy_hands()
    assert hands["left_hand"].cmd_topic == "rt/inspire_hand/ctrl/l"
    assert hands["left_hand"].state_topic == "rt/inspire_hand/state/l"
    assert hands["right_hand"].cmd_topic == "rt/inspire_hand/ctrl/r"
    assert hands["right_hand"].state_topic == "rt/inspire_hand/state/r"


def test_sdkpy_reads_angle_act_scaled(monkeypatch):
    _install_sdkpy(monkeypatch, {
        "rt/inspire_hand/state/l": _HandState([100, 200, 300, 400, 500, 600]),
        "rt/inspire_hand/state/r": _HandState([1000, 0, 0, 0, 0, 1000], err=[0, 0, 4, 0, 0, 0]),
    })
    hands = _sdkpy_hands(read_only=True)
    for h in hands.values():
        h.connect()
    assert np.allclose(hands["left_hand"].get_state().q, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    st = hands["right_hand"].get_state()
    assert np.allclose(st.q, [1, 0, 0, 0, 0, 1])
    assert st.extra["err"][2] == 4
    with pytest.raises(RuntimeError):
        hands["left_hand"].command(np.ones(6))


def test_sdkpy_command_sends_angle_mode_to_own_topic(monkeypatch):
    sent, _ = _install_sdkpy(monkeypatch, {
        "rt/inspire_hand/state/l": _HandState([500] * 6),
        "rt/inspire_hand/state/r": _HandState([500] * 6),
    })
    hands = _sdkpy_hands()
    for h in hands.values():
        h.connect()
    hands["left_hand"].command(np.array([1.5, 0.0, 0.5, 0.25, -0.2, 1.0]))
    assert len(sent) == 1  # one hand, one message; the other hand is untouched
    topic, msg = sent[-1]
    assert topic == "rt/inspire_hand/ctrl/l"
    assert msg.mode == 0b0001
    assert msg.angle_set == [1000, 0, 500, 250, 0, 1000]
    hands["right_hand"].reset()
    topic, msg = sent[-1]
    assert topic == "rt/inspire_hand/ctrl/r" and msg.angle_set == [500] * 6


def test_sdkpy_warns_when_bridge_lacks_angle_act(monkeypatch):
    _install_sdkpy(monkeypatch, {"rt/inspire_hand/state/l": _HandState([0] * 6)})
    h = _sdkpy_hands(read_only=True)["left_hand"]
    with pytest.warns(UserWarning, match="angle_act"):
        h.connect()


def test_sdkpy_connect_times_out_names_the_bridge(monkeypatch):
    _install_sdkpy(monkeypatch, {})
    h = _sdkpy_hands()["right_hand"]
    h._connect_timeout_s = 0.05
    with pytest.raises(TimeoutError, match="inspire_serial_to_dds"):
        h.connect()
