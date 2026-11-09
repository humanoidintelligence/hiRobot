"""``unitree_handcmd``: the Dex3-1 hand driver against a fake SDK."""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot.stamps import Stamp
from hirobot import load_embodiment
from hirobot.drivers.end_effector.unitree.sdk2_handcmd import motor_mode
from hirobot.end_effector import build_end_effectors
from tests.drivers.end_effector.unitree.sdk2_handcmd.fake_sdk import _Bus, _install_fake_sdk

CFG = "unitree_g1_dex3"


LEFT_Q = [0.35, 0.39, 0.65, -1.51, -1.63, -1.12, -1.43]   # read off the real left hand


RIGHT_Q = [0.50, -0.19, -1.49, 1.38, 1.68, 1.35, 1.71]


def _hands(monkeypatch, read_only=False):
    bus = _Bus()
    _install_fake_sdk(monkeypatch, bus, {"rt/dex3/left/state": LEFT_Q, "rt/dex3/right/state": RIGHT_Q})
    emb = load_embodiment(CFG)
    return emb, bus, build_end_effectors(emb, "eth0", read_only=read_only)


def test_motor_mode_byte_matches_sdk_example():
    assert motor_mode(0) == 0x10
    assert motor_mode(6) == 0x16
    assert motor_mode(3, timeout=True) == 0x93


def test_hand_reads_state_and_reports_no_torque(monkeypatch):
    _, _, hands = _hands(monkeypatch)
    left = hands["left_hand"]
    left.connect()
    s = left.get_state()
    assert np.allclose(s.q, LEFT_Q)
    assert np.all(np.isnan(s.tau))  # the raw tau_est is not in Nm
    assert s.extra["pressure"].shape == (9, 12)


def test_stale_state_raises(monkeypatch):
    _, _, hands = _hands(monkeypatch)
    left = hands["left_hand"]
    left.connect()
    left._stamp = Stamp(time.monotonic() - 5.0, 1)
    with pytest.raises(TimeoutError, match="old"):
        left.get_state()


def test_command_clips_to_position_limits(monkeypatch):
    _, bus, hands = _hands(monkeypatch)
    left = hands["left_hand"]
    left.connect()
    left.command(np.array([5.0, -5.0, 5.0, 5.0, -5.0, 0.5, -0.5]))
    modes, q, kp, kd = zip(*bus.sent["rt/dex3/left/cmd"][-1])
    assert np.allclose(q, [1.0472, -0.7243, 1.7453, 0.0, -1.7453, 0.0, -0.5])
    assert modes == tuple(0x10 + i for i in range(7))
    assert kp == (1.5,) * 7 and kd == (0.2,) * 7
    with pytest.raises(ValueError, match="finite"):
        left.command(np.full(7, np.nan))


def test_reset_ramps_kp_from_zero_at_measured_pose(monkeypatch):
    _, bus, hands = _hands(monkeypatch)
    right = hands["right_hand"]
    right._ramp_seconds, right._ramp_period = 0.05, 0.01
    right.connect()
    right.reset()
    sent = bus.sent["rt/dex3/right/cmd"]
    assert sent[0][0][2] == 0.0 and sent[-1][0][2] == 1.5
    for msg in sent:
        assert np.allclose([m[1] for m in msg], right.clip(np.array(RIGHT_Q)))


def test_damp_sends_zero_gains_with_timeout_flag(monkeypatch):
    _, bus, hands = _hands(monkeypatch)
    left = hands["left_hand"]
    left.connect()
    left.damp()
    for mode, q, kp, kd in bus.sent["rt/dex3/left/cmd"][-1]:
        assert mode & 0x80 and kp == 0.0 and kd == 0.0


def test_read_only_hand_never_publishes(monkeypatch):
    _, bus, hands = _hands(monkeypatch, read_only=True)
    left = hands["left_hand"]
    left.connect()
    assert left.get_state().q.shape == (7,)
    assert not bus.publishers
    with pytest.raises(RuntimeError, match="read_only"):
        left.command(np.zeros(7))
    with pytest.raises(RuntimeError, match="read_only"):
        left.reset()
    left.disconnect()  # damp is a no-op without a publisher
    assert not bus.publishers


def test_missing_state_times_out(monkeypatch):
    bus = _Bus()
    _install_fake_sdk(monkeypatch, bus)  # no state ever arrives
    emb = load_embodiment(CFG)
    left = build_end_effectors(emb)["left_hand"]
    t0 = time.time()
    with pytest.raises(TimeoutError, match="rt/dex3/left/state"):
        left.connect()
    assert time.time() - t0 < 3.0
