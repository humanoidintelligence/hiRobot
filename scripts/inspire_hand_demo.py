"""Open both Inspire hands slowly, hold, then return to the starting pose.

Works with either DDS bridge (``unitree_motorcmds`` or ``inspire_sdkpy``):

    # hiRobot bridge on PC2: scripts/inspire_serial_to_dds.py
    python scripts/inspire_hand_demo.py --iface eth0 --bridge inspire_sdkpy
    # Unitree bridge on PC2: h1_inspire_service (inspire_hand -s /dev/ttyUSB0)
    python scripts/inspire_hand_demo.py --iface eth0 --bridge unitree

Only the hands move; the body is not touched. Targets ramp linearly at
--hz, so no finger jumps. Ctrl-C during the motion sends the starting pose
back before exiting.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from hirobot import load_embodiment
from hirobot.devices.end_effector import build_end_effectors

CFG = "unitree_h1_inspire"


def ramp(hands, start, goal, seconds, hz):
    steps = max(1, int(seconds * hz))
    for k in range(1, steps + 1):
        a = k / steps
        for name, h in hands.items():
            h.command((1 - a) * start[name] + a * goal[name])
        time.sleep(1.0 / hz)


def show(hands, label):
    for name, h in hands.items():
        print(f"  {label:8s} {name:10s} q={np.round(h.get_state().q, 3).tolist()}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--iface", required=True, help="NIC on 192.168.123.0/24, e.g. eth0")
    ap.add_argument("--bridge", choices=["inspire_sdkpy", "unitree"], required=True,
                    help="inspire_sdkpy: scripts/inspire_serial_to_dds.py; "
                         "unitree: h1_inspire_service 1.0.1 (the config's default)")
    ap.add_argument("--seconds", type=float, default=3.0, help="duration of each ramp")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold fully open")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--config", default=CFG)
    args = ap.parse_args()

    emb = load_embodiment(args.config)
    if args.bridge == "inspire_sdkpy":  # the config uses the Unitree service
        for ee in emb.end_effectors:
            if ee.driver:
                ee.driver = {"type": "inspire_sdkpy", "side": ee.name.split("_")[0]}
    hands = build_end_effectors(emb, args.iface)
    for h in hands.values():
        h.connect()
    time.sleep(0.3)

    start = {n: h.get_state().q.copy() for n, h in hands.items()}
    opened = {n: np.ones(6) for n in hands}
    show(hands, "start")
    try:
        print(f"opening over {args.seconds:.1f} s")
        ramp(hands, start, opened, args.seconds, args.hz)
        time.sleep(args.hold)
        show(hands, "open")
        print(f"returning over {args.seconds:.1f} s")
        ramp(hands, opened, start, args.seconds, args.hz)
    except KeyboardInterrupt:
        print("interrupted: sending the starting pose")
        for n, h in hands.items():
            h.command(start[n])
    time.sleep(1.0)
    show(hands, "end")
    for n, h in hands.items():
        err = np.abs(h.get_state().q - start[n]).max()
        print(f"  {n}: max |end - start| = {err:.3f}")


if __name__ == "__main__":
    main()
