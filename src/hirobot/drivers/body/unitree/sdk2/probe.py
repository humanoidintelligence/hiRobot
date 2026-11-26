"""Read-only probe: which Unitree humanoid is on a network interface.

:func:`probe` joins the robot's DDS domain on one NIC and only listens: it
subscribes to ``rt/lowstate`` (``unitree_hg`` and ``unitree_go`` types at once;
a sample of one family never reaches a reader of the other) and to
``rt/dex3/<side>/state``, and asks the motion switcher ``CheckMode`` (a
read-only query). It never creates a ``ChannelPublisher``, never calls
``SelectMode`` / ``ReleaseMode``, and never sends a command.

:func:`candidates` turns what was seen into built-in robot configs to offer,
ranked by confidence. A thing not seen is reported as unknown (``None``), not
as absent: a G1 with a locked waist reports 27 motors, and a hand that is
switched off is silent on DDS.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from hirobot.drivers._sdk import unitree_sdk2
from hirobot.loading.sdk import require_sdk

_LOWSTATE_TOPIC = "rt/lowstate"
_HAND_TOPICS = {"dex3_left": "rt/dex3/left/state", "dex3_right": "rt/dex3/right/state"}
# Motors of each built-in model (its body's joint count; tests check them against
# load_embodiment). The legacy H1 is the only go-family robot.
_MOTORS = {"robots/unitree_g1": 29, "robots/unitree_h1_2": 27, "robots/unitree_h2": 31,
           "robots/unitree_h1": 19}
_G1, _G1_DEX3 = "robots/unitree_g1", "robots/unitree_g1_dex3"
_H1, _H1_2, _H2 = "robots/unitree_h1", "robots/unitree_h1_2", "robots/unitree_h2"
# A G1 29-DoF with its waist locked reads 0 temperature and 0 voltage on these slots.
_G1_WAIST_SLOTS = {13, 14}
# mode_machine read on a G1 29-DoF (2026-11). Other models' values are not known yet.
_G1_MODE_MACHINE = 5
_SAMPLE_S = 0.5  # after the first LowState: how long to count messages for lowstate_hz
_HAND_WINDOW_S = 0.5  # least time the Dex3 topics are listened to
_CHECK_MODE_TIMEOUT_S = 1.0


def _empty(interface: Optional[str]) -> Dict[str, Any]:
    return {
        "interface": interface,
        "status": "not_seen",
        "error": None,
        "msg_family": None,
        "motor_slots": None,
        "motors_reporting": None,
        "reporting_slots": None,
        "mode_machine": None,
        "lowstate_hz": None,
        "motion_service": {"checked": False, "mode": None, "owns_lowcmd": None},
        "hands": {name: None for name in _HAND_TOPICS},
        "candidates": [],
        "notes": [],
    }


def _error(report: Dict[str, Any], message: str) -> Dict[str, Any]:
    report["status"] = "error"
    report["error"] = message
    return report


class _Listener:
    """Counts the samples of one subscription and keeps the first and the newest."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.count = 0
        self.first_t: Optional[float] = None
        self.last = None

    def __call__(self, msg) -> None:
        with self.lock:
            if self.first_t is None:
                self.first_t = time.monotonic()
            self.count += 1
            self.last = msg


def probe(interface: str, timeout: float = 2.0, domain_id: int = 0) -> Dict[str, Any]:
    """Listen on ``interface`` for up to ``timeout`` s and describe the Unitree robot there.

    Returns the ``unitree`` object of a scan report: ``status`` is ``"found"``
    when ``rt/lowstate`` arrived, ``"not_seen"`` when nothing did, ``"error"``
    when the SDK is missing or DDS could not start on the NIC. Never raises
    for those; never commands the robot.
    """
    report = _empty(interface)
    try:
        require_sdk(__package__, "Unitree probe")
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as GoLowState
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState
    except ImportError as exc:
        return _error(report, f"unitree_sdk2py not usable ({exc}); install: "
                              f"{unitree_sdk2.INSTALL}")

    first = unitree_sdk2._initialised
    if first is not None and first != (domain_id, interface):
        # The SDK's channel factory is process-wide: a second NIC would be ignored.
        return _error(report, f"DDS already runs on {first[1]!r} (domain {first[0]}) in this "
                              f"process; probe {interface!r} from a fresh process")
    try:
        unitree_sdk2.init_channel("Unitree probe", domain_id, interface)
    except Exception as exc:  # noqa: BLE001 - reported, never raised
        return _error(report, f"{type(exc).__name__}: {exc}")

    subs: List[Any] = []

    def listen(topic, msg_type) -> Tuple[Any, _Listener]:
        listener = _Listener()
        sub = ChannelSubscriber(topic, msg_type)
        subs.append(sub)
        sub.Init(listener, 0)
        return sub, listener

    try:
        start = time.monotonic()
        hands = {name: listen(topic, HandState_)[1] for name, topic in _HAND_TOPICS.items()}
        hg_sub, hg = listen(_LOWSTATE_TOPIC, HgLowState)
        families = {"hg": hg}
        try:
            # cyclonedds takes both types on one topic in one participant, and an hg
            # sample never reaches the go reader (checked on loopback), so both
            # families are listened to at once.
            families["go"] = listen(_LOWSTATE_TOPIC, GoLowState)[1]
            together = True
        except Exception:  # noqa: BLE001 - a DDS that refuses two types: go after hg
            together = False
        family = _wait_lowstate(families, timeout)
        if family is None and not together:
            _close([hg_sub])
            subs.remove(hg_sub)
            families = {"go": listen(_LOWSTATE_TOPIC, GoLowState)[1]}
            family = _wait_lowstate(families, timeout)
        if family is not None:
            got = families[family]
            time.sleep(max(0.0, min(_SAMPLE_S, timeout) - (time.monotonic() - got.first_t)))
            _read_lowstate(report, family, got)
            report["motion_service"] = _check_mode(min(_CHECK_MODE_TIMEOUT_S, timeout))
        time.sleep(max(0.0, min(_HAND_WINDOW_S, timeout) - (time.monotonic() - start)))
        for name, listener in hands.items():
            report["hands"][name] = True if listener.count else None
    finally:
        _close(subs)

    if report["status"] == "found":
        cands, notes = _rank(report)
        report["candidates"] = cands
        report["notes"].extend(notes)
    else:
        report["notes"].append(
            f"no rt/lowstate on {interface} in {timeout:g} s: robot off, wrong NIC, or another "
            "DDS domain; please confirm")
    return report


def _wait_lowstate(families: Dict[str, _Listener], timeout: float) -> Optional[str]:
    deadline = time.monotonic() + timeout
    while True:
        for family, listener in families.items():
            if listener.count:
                return family
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.01)


def _close(subs) -> None:
    for sub in subs:
        try:
            sub.Close()
        except Exception:  # noqa: BLE001 - best effort; the process owns the DDS entities
            pass


def _reports(motor) -> bool:
    """A motor slot is live when it reads a temperature or a voltage (q if it has neither)."""
    temp = getattr(motor, "temperature", None)
    vol = getattr(motor, "vol", None)
    if temp is None and vol is None:
        return bool(getattr(motor, "q", 0.0))
    temps = temp if hasattr(temp, "__len__") else [temp]
    return any(t for t in temps if t is not None) or bool(vol)


def _read_lowstate(report: Dict[str, Any], family: str, got: _Listener) -> None:
    with got.lock:
        msg, count, first_t = got.last, got.count, got.first_t
    elapsed = time.monotonic() - first_t
    motors = list(msg.motor_state)
    live = [i for i, m in enumerate(motors) if _reports(m)]
    report.update(
        status="found",
        msg_family=family,
        motor_slots=len(motors),
        motors_reporting=len(live),
        reporting_slots=live,
        mode_machine=int(msg.mode_machine) if family == "hg" else None,
        lowstate_hz=round((count - 1) / elapsed, 1) if count > 1 and elapsed > 0 else None,
    )


def _check_mode(timeout: float) -> Dict[str, Any]:
    """Ask the motion switcher which built-in service runs (``CheckMode``, read-only)."""
    unknown = {"checked": False, "mode": None, "owns_lowcmd": None}
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        client = MotionSwitcherClient()
        client.SetTimeout(timeout)
        client.Init()
        code, result = client.CheckMode()
    except Exception:  # noqa: BLE001 - the switcher is optional on some firmware
        return unknown
    if code != 0 or not isinstance(result, dict):
        return unknown
    name = str(result.get("name") or "")
    return {"checked": True, "mode": name, "owns_lowcmd": bool(name)}


def candidates(unitree: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Built-in configs that fit a probe result, most likely first.

    Each entry is ``{"config", "confidence", "why"}``. Pure: reads only the
    dict (``msg_family``, ``motors_reporting``, ``reporting_slots`` when
    present, ``mode_machine``, ``hands``). Empty unless ``status`` is ``"found"``.
    """
    return _rank(unitree)[0]


def _cand(config: str, confidence: float, why: str) -> Dict[str, Any]:
    return {"config": config, "confidence": round(confidence, 2), "why": why}


def _rank(u: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[str]]:
    """``(candidates, notes)`` for a probe result."""
    if u.get("status") != "found":
        return [], []
    family = u.get("msg_family")
    total = u.get("motors_reporting")
    slots = u.get("reporting_slots")
    live = set(slots) if slots is not None else None
    notes: List[str] = []

    def reporting(config: str) -> Optional[int]:
        """Live motors among the model's own slots (all live motors when slots are unknown)."""
        if live is None:
            return total
        return sum(1 for i in live if i < _MOTORS[config])

    if family == "go":
        n = reporting(_H1)
        why = "unitree_go LowState: only the 19-DoF H1 uses that message family"
        if n != _MOTORS[_H1]:
            notes.append(f"{n} of {_MOTORS[_H1]} H1 motors report; please confirm the model")
            return [_cand(_H1, 0.7, why)], notes
        return [_cand(_H1, 0.9, why)], notes
    if family != "hg":
        return [], [f"unknown message family {family!r}; please confirm the model"]

    mode_machine = u.get("mode_machine")
    out: List[Dict[str, Any]] = []
    if reporting(_H2) == _MOTORS[_H2]:
        out.append(_cand(_H2, 0.8, "unitree_hg with 31 motors reporting: H2"))
    elif reporting(_G1) == _MOTORS[_G1]:
        out.append(_cand(_G1, 0.85, "unitree_hg with 29 motors reporting: G1 29-DoF"))
    elif reporting(_G1) == 27 or reporting(_H1_2) == 27:
        g1_waist = live is not None and set(range(29)) - live == _G1_WAIST_SLOTS
        h1_2_slots = live is not None and live == set(range(27))
        if mode_machine == _G1_MODE_MACHINE or (g1_waist and not h1_2_slots):
            g1, h12 = 0.6, 0.4
        else:
            g1, h12 = 0.4, 0.6
        detail = f"mode_machine {mode_machine}" if mode_machine is not None else "no mode_machine"
        g1_detail = detail + ("; waist slots 13, 14 silent" if g1_waist else "")
        h12_detail = detail + ("; slots 0-26 live" if h1_2_slots else "")
        out.append(_cand(_H1_2, h12, f"unitree_hg with 27 motors reporting: H1-2 ({h12_detail})"))
        out.append(_cand(_G1, g1, "unitree_hg with 27 motors reporting: G1 29-DoF with a "
                                  f"locked waist ({g1_detail})"))
        notes.append("only 27 motors report: G1 with a locked waist, or H1-2; please confirm")
    else:
        for config in (_G1, _H1_2, _H2):
            out.append(_cand(config, 0.2, f"unitree_hg with {total} motors reporting; "
                                          f"{_MOTORS[config]} expected"))
        notes.append(f"{total} motors report, which fits no built-in hg model; "
                     "please confirm the model")

    hands = u.get("hands") or {}
    left, right = hands.get("dex3_left"), hands.get("dex3_right")
    g1 = next((c for c in out if c["config"] == _G1), None)
    if g1 is not None and left and right:
        out.append(_cand(_G1_DEX3, min(g1["confidence"] + 0.05, 0.95),
                         g1["why"] + "; rt/dex3/left/state and rt/dex3/right/state seen"))
    elif left or right:
        side = "left" if left else "right"
        notes.append(f"only the {side} Dex3 hand was seen; the other may be off or "
                     "unplugged, please confirm")
    if (u.get("motion_service") or {}).get("owns_lowcmd"):
        notes.append(f"built-in motion service {u['motion_service']['mode']!r} owns rt/lowcmd; "
                     "joint commands need it released (robot hung or supported)")
    out.sort(key=lambda c: c["confidence"], reverse=True)  # stable: ties keep rule order
    return out, notes
