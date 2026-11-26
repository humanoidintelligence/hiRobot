"""Read-only Unitree probe (``probe`` / ``candidates``) against the fake SDK."""

from __future__ import annotations

import builtins
import time

import pytest

from hirobot import load_embodiment
from hirobot.drivers.body.unitree.sdk2 import probe as probe_mod
from hirobot.drivers.body.unitree.sdk2.probe import candidates, probe
from tests.drivers.body.unitree.sdk2 import fake_sdk

DEX3 = {"rt/dex3/left/state": fake_sdk.hand_state(), "rt/dex3/right/state": fake_sdk.hand_state()}
KEYS = {"interface", "status", "error", "msg_family", "motor_slots", "motors_reporting",
        "reporting_slots", "mode_machine", "lowstate_hz", "motion_service", "hands", "candidates", "notes"}


@pytest.fixture(autouse=True)
def _short_windows(monkeypatch):
    monkeypatch.setattr(probe_mod, "_SAMPLE_S", 0.02)
    monkeypatch.setattr(probe_mod, "_HAND_WINDOW_S", 0.02)


def _assert_read_only(bus):
    assert bus.publishers == []
    assert "SelectMode" not in bus.switcher_calls
    assert "ReleaseMode" not in bus.switcher_calls
    # Every subscriber the probe opened was closed again.
    assert sorted(bus.closed) == sorted(t for t, _ in bus.subscribed)


def _configs(report):
    return [c["config"] for c in report["candidates"]]


def test_model_motor_counts_match_the_builtin_configs():
    for config, n in probe_mod._MOTORS.items():
        assert load_embodiment(config).num_dofs == n, config
    assert load_embodiment("robots/unitree_g1_dex3").num_dofs == 29 + 14


def test_g1_29_motors(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29, reporting=range(29)),
                           mode="ai")
    r = probe("eth0", timeout=0.5)
    assert set(r) == KEYS
    assert r["status"] == "found" and r["error"] is None
    assert r["msg_family"] == "hg" and r["motor_slots"] == 35
    assert r["motors_reporting"] == 29 and r["mode_machine"] == 5
    assert r["motion_service"] == {"checked": True, "mode": "ai", "owns_lowcmd": True}
    assert r["hands"] == {"dex3_left": None, "dex3_right": None}
    assert _configs(r)[0] == "robots/unitree_g1"
    assert "robots/unitree_g1_dex3" not in _configs(r)
    assert bus.dds == [(0, "eth0")]
    _assert_read_only(bus)


def test_g1_with_both_dex3_hands(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29, reporting=range(29)),
                           hands=DEX3)
    r = probe("eth0", timeout=0.5)
    assert r["hands"] == {"dex3_left": True, "dex3_right": True}
    assert _configs(r)[:2] == ["robots/unitree_g1_dex3", "robots/unitree_g1"]
    assert r["motion_service"] == {"checked": True, "mode": "", "owns_lowcmd": False}
    _assert_read_only(bus)


def test_27_motors_offers_g1_and_h1_2_with_a_note(monkeypatch):
    live = [i for i in range(29) if i not in (13, 14)]  # G1 with a locked waist
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29, reporting=live))
    r = probe("eth0", timeout=0.5)
    assert r["motors_reporting"] == 27
    assert set(_configs(r)) == {"robots/unitree_g1", "robots/unitree_h1_2"}
    assert any("27 motors" in n and "please confirm" in n for n in r["notes"])
    _assert_read_only(bus)


def test_go_family_is_the_h1(monkeypatch):
    h1_slots = [i for i in range(20) if i != 9]
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 20, slots=20,
                                                           reporting=h1_slots))
    r = probe("eth0", timeout=0.5)
    assert r["msg_family"] == "go" and r["motor_slots"] == 20
    assert r["motors_reporting"] == 19 and r["mode_machine"] is None
    assert _configs(r) == ["robots/unitree_h1"]
    _assert_read_only(bus)


def test_silent_bus_is_not_seen(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29), hands=DEX3,
                           silent={"rt/lowstate", "rt/dex3/left/state", "rt/dex3/right/state"})
    t0 = time.monotonic()
    r = probe("eth0", timeout=0.2)
    assert time.monotonic() - t0 < 2.0
    assert r["status"] == "not_seen" and r["candidates"] == []
    assert r["hands"] == {"dex3_left": None, "dex3_right": None}
    assert r["motion_service"]["checked"] is False
    assert any("no rt/lowstate on eth0" in n and "please confirm" in n for n in r["notes"])
    assert bus.switcher_calls == []
    _assert_read_only(bus)


def test_switcher_without_answer_is_unchecked(monkeypatch):
    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29, reporting=range(29)),
                           mode=None)
    r = probe("eth0", timeout=0.5)
    assert r["motion_service"] == {"checked": False, "mode": None, "owns_lowcmd": None}
    _assert_read_only(bus)


def test_sdk_missing_is_an_error_not_an_exception(monkeypatch):
    real_import = builtins.__import__

    def no_sdk(name, *args, **kwargs):
        if name.startswith("unitree_sdk2py"):
            raise ImportError(f"No module named {name!r}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(probe_mod, "require_sdk", lambda *a: {})
    monkeypatch.setattr(builtins, "__import__", no_sdk)
    r = probe("eth0", timeout=0.2)
    assert r["status"] == "error" and "unitree_sdk2py" in r["error"]
    assert "pip install" in r["error"]
    assert r["candidates"] == [] and KEYS <= set(r)


def test_require_sdk_failure_is_an_error(monkeypatch):
    def missing(*_a):
        raise ImportError("unitree_sdk2py is not installed.")

    monkeypatch.setattr(probe_mod, "require_sdk", missing)
    r = probe("eth0", timeout=0.2)
    assert r["status"] == "error" and "not installed" in r["error"]


def test_dds_init_failure_is_an_error(monkeypatch):
    fake_sdk.install(monkeypatch, None)
    import unitree_sdk2py.core.channel as channel

    def fail(*_a):
        raise Exception("channel factory init error.")

    monkeypatch.setattr(channel, "ChannelFactoryInitialize", fail)
    r = probe("nosuchnic0", timeout=0.2)
    assert r["status"] == "error" and "nosuchnic0" in r["error"]


# ------------------------------------------------------------------ candidates()


def _found(family="hg", n=None, slots=None, mode_machine=5, left=None, right=None):
    if slots is not None:
        n = len(slots)
    return {"status": "found", "msg_family": family, "motors_reporting": n,
            "reporting_slots": slots, "mode_machine": mode_machine,
            "hands": {"dex3_left": left, "dex3_right": right}}


def test_candidates_need_a_found_robot():
    assert candidates({"status": "not_seen"}) == []
    assert candidates({"status": "error"}) == []


def test_candidates_g1():
    c = candidates(_found(n=29))
    assert c[0]["config"] == "robots/unitree_g1" and c[0]["why"]


def test_candidates_g1_dex3_needs_both_hands():
    assert candidates(_found(n=29, left=True, right=True))[0]["config"] == \
        "robots/unitree_g1_dex3"
    one = candidates(_found(n=29, left=True))
    assert [c["config"] for c in one] == ["robots/unitree_g1"]


def test_candidates_h2():
    assert candidates(_found(n=31))[0]["config"] == "robots/unitree_h2"
    # 31 live slots on an H2 must not be counted as a G1 (slots < 29 only).
    assert "robots/unitree_g1" not in [c["config"] for c in candidates(_found(slots=range(31)))]


def test_candidates_27_motors():
    # Slots 0-26 live, mode_machine unknown: H1-2 first.
    c = candidates(_found(slots=list(range(27)), mode_machine=None))
    assert [x["config"] for x in c] == ["robots/unitree_h1_2", "robots/unitree_g1"]
    # The G1 waist slots silent and the G1's mode_machine: G1 first.
    g1 = candidates(_found(slots=[i for i in range(29) if i not in (13, 14)]))
    assert [x["config"] for x in g1] == ["robots/unitree_g1", "robots/unitree_h1_2"]
    # A count alone, with no mode_machine: H1-2 first, G1 still offered.
    bare = candidates(_found(n=27, mode_machine=None))
    assert [x["config"] for x in bare] == ["robots/unitree_h1_2", "robots/unitree_g1"]


def test_candidates_go_and_unknown_counts():
    assert [c["config"] for c in candidates(_found("go", n=19, mode_machine=None))] == \
        ["robots/unitree_h1"]
    odd = candidates(_found(n=12))
    assert {c["config"] for c in odd} == {"robots/unitree_g1", "robots/unitree_h1_2",
                                          "robots/unitree_h2"}
    assert all(c["confidence"] <= 0.3 for c in odd)


def test_candidates_are_sorted_by_confidence():
    for u in (_found(n=29, left=True, right=True), _found(n=27), _found(n=5)):
        conf = [c["confidence"] for c in candidates(u)]
        assert conf == sorted(conf, reverse=True)
