"""``unitree_sdk2`` motor slot mapping (the legacy H1 skips slot 9)."""

from __future__ import annotations

import pytest

from hirobot import load_embodiment
from hirobot.registry import get_body


def test_h1_go_skips_slot_9():
    emb = load_embodiment("unitree_h1")
    r = get_body("unitree_sdk2")(emb, msg_family="go")
    assert r.motor_index == [0, 1, 2, 3, 4, 5, 6, 7, 8] + list(range(10, 20))
    # Ankles and arms (slots 10-19) are the H1's "weak" motors.
    assert r._go_mode == [0x0A] * 9 + [0x01] * 10
    # hg robots keep 0..n-1.
    g1 = get_body("unitree_sdk2")(load_embodiment("unitree_g1"))
    assert g1.motor_index == list(range(29))


def test_motor_index_validated():
    emb = load_embodiment("unitree_h1")
    with pytest.raises(ValueError, match="entries"):
        get_body("unitree_sdk2")(emb, msg_family="go", motor_index=[0, 1])
    with pytest.raises(ValueError, match="distinct"):
        get_body("unitree_sdk2")(emb, msg_family="go", motor_index=[0] * 19)
