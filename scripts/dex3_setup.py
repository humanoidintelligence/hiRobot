"""Unitree Dex3-1 hand setup checks (G1). The body is never commanded.

Both steps build only the hand drivers from the config's end-effectors, so no
``rt/lowcmd`` publisher exists in this process.

``status`` (read-only, publishes nothing)
    Reads ``rt/dex3/<side>/state`` for a few seconds and prints the state rate,
    per-joint position (and where it sits in its limit range), motor
    temperature and the pressure-sensor summary.

``wiggle`` (commands the hands; keep the fingers clear)
    For each selected hand: ramp kp from 0 holding the measured pose, move all
    7 joints on a minimum-jerk path to the goal, hold, move back, then damp.
    The goal is a fraction of the way towards mid-range (``--pose mid``) or
    every finger fully open (``--pose open``: all joints 0, kept 0.05 rad off
    a hard stop). Aborts to damping if a joint lags
    its path by more than ``--max-track-error`` or the state goes stale.

::

    python scripts/dex3_setup.py status --config unitree_g1_dex3 --iface eth0
    python scripts/dex3_setup.py wiggle --config unitree_g1_dex3 --iface eth0 \\
        --hand left --fraction 0.3
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from hirobot import load_embodiment
from hirobot.devices.end_effector import build_end_effectors


def _min_jerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def _hands(args, read_only: bool):
    emb = load_embodiment(args.config)
    hands = build_end_effectors(emb, args.iface, read_only=read_only)
    if not hands:
        raise SystemExit(f"{args.config} declares no end-effector with a hand driver")
    if getattr(args, "hand", "both") != "both":
        hands = {k: h for k, h in hands.items() if k.startswith(args.hand)}
    return hands


def cmd_status(args) -> int:
    hands = _hands(args, read_only=True)
    for hand in hands.values():
        hand.connect()
    stamps = {k: [] for k in hands}
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        for k, h in hands.items():
            s = h.get_state().stamp.t
            if not stamps[k] or s != stamps[k][-1]:
                stamps[k].append(s)
        time.sleep(0.0005)
    for k, h in hands.items():
        st, raw = h.get_state(), h.raw_state
        print(f"{k} ({h.state_topic}): ~{len(stamps[k]) / args.seconds:.0f} Hz sampled")
        print(f"  {'joint':<22}{'q(rad)':>9}{'in range':>10}{'temp(C)':>9}")
        for i, j in enumerate(h.joints):
            lo, hi = j.position_limit
            frac = (st.q[i] - lo) / (hi - lo)
            temp = max(raw.motor_state[i].temperature)
            print(f"  {j.name:<22}{st.q[i]:>9.3f}{frac:>10.2f}{temp:>9}")
        p = st.extra.get("pressure")
        if p is not None:
            print(f"  pressure: {p.shape[0]} sensors x {p.shape[1]}, min {p.min():.0f} max {p.max():.0f}")
    return 0


#: Margin kept from a hard stop when a pose target sits on a joint limit (rad).
_LIMIT_MARGIN = 0.05


def open_pose(hand) -> np.ndarray:
    """All joints at 0: fingers straight, thumb out (farthest tips in the G1 MJCF).

    For index, middle and thumb_2, 0 is the end of the range; those targets
    stay ``_LIMIT_MARGIN`` inside it so the fingers do not press on the stop.
    """
    lo = np.array([j.position_limit[0] for j in hand.joints])
    hi = np.array([j.position_limit[1] for j in hand.joints])
    return np.clip(np.zeros(hand.num_joints), lo + _LIMIT_MARGIN, hi - _LIMIT_MARGIN)


def _wiggle_one(hand, args) -> int:
    lo = np.array([j.position_limit[0] for j in hand.joints])
    hi = np.array([j.position_limit[1] for j in hand.joints])
    start = hand.clip(hand.get_state().q)
    if args.pose == "open":
        goal = open_pose(hand)
    else:
        goal = start + args.fraction * ((lo + hi) / 2 - start)
    travel = np.abs(goal - start)
    print(f"{hand.name}: start {np.round(start, 3).tolist()}")
    print(f"{hand.name}: goal  {np.round(goal, 3).tolist()} (max travel {travel.max():.3f} rad)")
    print(f"{hand.name}: ramping kp 0 -> default over the reset window ...")
    hand.reset()
    period = 1.0 / args.rate
    worst = 0.0
    for phase, dur, a, b in (("out", args.duration, start, goal), ("hold", args.hold, goal, goal),
                             ("back", args.duration, goal, start), ("settle", 0.5, start, start)):
        ts = time.time()
        next_print = ts
        while True:
            now = time.time()
            s = (now - ts) / dur
            desired = a + (b - a) * _min_jerk(s)
            hand.command(desired)
            q = hand.get_state().q  # raises if the state goes stale
            err = np.abs(q - desired)
            worst = max(worst, float(err.max()))
            if err.max() > args.max_track_error:
                k = int(err.argmax())
                print(f"ABORT: {hand.joints[k].name} lags its path by {err[k]:.3f} rad; damping.")
                return 3
            if now >= next_print:
                print(f"  {now - ts:5.2f}s {phase:<7} max err {err.max():.3f} "
                      f"({hand.joints[int(err.argmax())].name})")
                next_print += 0.5
            if s >= 1.0:
                break
            time.sleep(period)
    print(f"{hand.name}: done, worst tracking error {worst:.3f} rad")
    return 0


def cmd_wiggle(args) -> int:
    if not 0.0 < args.fraction <= 0.6:
        print("--fraction must be in (0, 0.6]")
        return 2
    hands = _hands(args, read_only=False)
    connected = []
    try:
        for hand in hands.values():
            hand.connect()
            connected.append(hand)
        for hand in connected:
            rc = _wiggle_one(hand, args)
            hand.damp()
            if rc:
                return rc
        return 0
    finally:
        for hand in connected:
            hand.disconnect()  # damps: zero gains, timeout flag
        print("Hands left damped (kp = kd = 0).")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("status", "wiggle"):
        sp = sub.add_parser(name)
        sp.add_argument("--config", required=True, help="Config whose end-effectors declare hand drivers.")
        sp.add_argument("--iface", required=True, help="Host NIC on the robot subnet, e.g. eth0.")
        sp.add_argument("--hand", choices=["left", "right", "both"], default="both")
    sub.choices["status"].add_argument("--seconds", type=float, default=2.0)
    w = sub.choices["wiggle"]
    w.add_argument("--pose", choices=["mid", "open"], default="mid",
                   help="mid: --fraction of the way towards mid-range; open: all fingers fully open.")
    w.add_argument("--fraction", type=float, default=0.3,
                   help="How far towards the middle of each joint's range to move (0-0.6).")
    w.add_argument("--duration", type=float, default=3.0, help="Seconds for the move out (and back).")
    w.add_argument("--hold", type=float, default=1.0, help="Seconds to hold at the goal.")
    w.add_argument("--rate", type=float, default=100.0, help="Command rate (Hz).")
    w.add_argument("--max-track-error", type=float, default=0.4, help="Abort past this lag (rad).")
    args = p.parse_args(argv)
    return {"status": cmd_status, "wiggle": cmd_wiggle}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
