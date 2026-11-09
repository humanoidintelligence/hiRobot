"""Software e-stop for Unitree humanoids. Run it in its own terminal.

While armed it only *listens* to ``rt/lowstate`` and prints a heartbeat; it
publishes nothing. On Ctrl-C (SIGINT) or SIGTERM it immediately:

1. sends SIGINT to controller processes whose command line matches ``--kill``
   (default ``unitree_setup.py``), so they stop publishing and run their own
   shutdown;
2. publishes a damping ``LowCmd`` (kp = 0, kd = ``--kd``, no torque) on
   ``rt/lowcmd`` at ``--rate`` Hz, and keeps doing so;
3. if a built-in motion service (``ai`` / locomotion) is active, asks it to
   enter its own damping state through the loco client.

Press Ctrl-C a second time to exit. Once this script exits nothing keeps the
robot damped, so leave it running until the robot is supported or powered down.

This is a software stop: it needs the DDS link to work. Any other process that
keeps publishing ``rt/lowcmd`` will interleave with the damping commands. Keep
the robot's physical e-stop or the remote within reach.

Usage::

    python scripts/unitree_estop.py --iface eth0               # G1 / H1-2 / H2
    python scripts/unitree_estop.py --iface eth0 --msg-family go   # legacy H1
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

# Legacy H1 (go family) uses mode 0x01 on its "weak" motors (ankles, arms) and
# 0x0A elsewhere, per unitree_sdk2_python example/h1/low_level.
_H1_WEAK_MOTORS = set(range(10, 20))


class EStop:
    def __init__(self, args) -> None:
        self.args = args
        self.state = None
        self.state_time = 0.0
        self.mode_machine = 0
        self.triggered = threading.Event()
        self.exit_requested = threading.Event()
        self.sent = 0

    # ----- setup -------------------------------------------------------------
    def connect(self) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.utils.crc import CRC

        if self.args.msg_family == "hg":
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        else:
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_ as LowCmdDefault
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_

        ChannelFactoryInitialize(self.args.domain_id, self.args.iface)
        # Create the publisher now so triggering costs no setup time. Creating a
        # writer sends nothing; only Write() does.
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        self.cmd = LowCmdDefault()
        self.crc = CRC()

    def _on_state(self, msg) -> None:
        self.state = msg
        self.state_time = time.time()
        self.mode_machine = int(getattr(msg, "mode_machine", 0))

    # ----- damping command ---------------------------------------------------
    def _fill_damping(self) -> None:
        cmd = self.cmd
        if self.args.msg_family == "hg":
            cmd.mode_pr = 0
            cmd.mode_machine = self.mode_machine
        else:
            cmd.head[0], cmd.head[1] = 0xFE, 0xEF
            cmd.level_flag = 0xFF
        for i, m in enumerate(cmd.motor_cmd):
            if self.args.msg_family == "hg":
                m.mode = 1
            else:
                m.mode = 0x01 if i in _H1_WEAK_MOTORS else 0x0A
            m.q = 0.0
            m.dq = 0.0
            m.kp = 0.0
            m.kd = self.args.kd
            m.tau = 0.0
        cmd.crc = self.crc.Crc(cmd)

    def _damping_loop(self) -> None:
        period = 1.0 / self.args.rate
        while not self.exit_requested.is_set():
            self._fill_damping()  # refresh mode_machine + CRC each time
            self.pub.Write(self.cmd)
            self.sent += 1
            time.sleep(period)

    # ----- trigger actions ---------------------------------------------------
    def _stop_controllers(self) -> None:
        if not self.args.kill:
            return
        try:
            out = subprocess.run(
                ["pgrep", "-f", self.args.kill], capture_output=True, text=True, check=False
            )
        except FileNotFoundError:
            print("  [kill] pgrep not found; skipping")
            return
        me = os.getpid()
        pids = [int(p) for p in out.stdout.split() if p.strip() and int(p) != me]
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
                print(f"  [kill] SIGINT -> pid {pid}")
            except ProcessLookupError:
                pass
        if not pids:
            print(f"  [kill] no process matching {self.args.kill!r}")

    def _builtin_damp(self) -> None:
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )

            msc = MotionSwitcherClient()
            msc.SetTimeout(1.0)
            msc.Init()
            code, result = msc.CheckMode()
            name = (result or {}).get("name") if code == 0 else None
            if not name:
                print(f"  [builtin] no built-in motion service active (code={code})")
                return
            if self.args.msg_family == "hg":
                from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            else:
                from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
            loco = LocoClient()
            loco.SetTimeout(1.0)
            loco.Init()
            loco.Damp()
            print(f"  [builtin] {name!r} active -> loco Damp() sent")
        except Exception as exc:  # noqa: BLE001 - best effort; low-level damping already runs
            print(f"  [builtin] damp request failed: {exc}")

    def trigger(self) -> None:
        if self.triggered.is_set():
            return
        self.triggered.set()
        t0 = time.time()
        # Low-level damping first: it needs no round trip.
        threading.Thread(target=self._damping_loop, daemon=True).start()
        print(f"\n!!! E-STOP {time.strftime('%H:%M:%S')}: damping kd={self.args.kd} at {self.args.rate:.0f} Hz")
        if self.state is None:
            print("  WARNING: never received lowstate; sending damping with mode_machine=0 anyway")
        self._stop_controllers()
        threading.Thread(target=self._builtin_damp, daemon=True).start()
        print(f"  first damping command within {1000 * (time.time() - t0):.1f} ms of the signal")
        print("  Ctrl-C again to exit (the robot is then no longer held in damping by this script).")

    # ----- main loop ---------------------------------------------------------
    def run(self) -> int:
        def on_signal(signum, _frame):
            if self.triggered.is_set():
                self.exit_requested.set()
            else:
                self.trigger()

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        print(f"E-stop ARMED on {self.args.iface} ({self.args.msg_family}). Listening only; publishing nothing.")
        print("Press Ctrl-C to damp the robot.")
        last_print = 0.0
        while not self.exit_requested.is_set():
            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                if self.triggered.is_set():
                    line = f"E-STOP ACTIVE  damping cmds sent: {self.sent}"
                elif self.state is None:
                    line = "armed  WAITING for rt/lowstate (check --iface / cable)"
                else:
                    age = now - self.state_time
                    tag = "ok" if age < 0.2 else f"STALE {age:.1f}s"
                    line = f"armed  lowstate {tag}  mode_machine={self.mode_machine}"
                print(f"\r{line:<70}", end="", flush=True)
            time.sleep(0.02)
        print(f"\nExiting. Sent {self.sent} damping commands.")
        return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Software e-stop for Unitree humanoids (Ctrl-C to damp).")
    p.add_argument("--iface", required=True, help="Host NIC on the robot subnet, e.g. eth0.")
    p.add_argument("--msg-family", choices=["hg", "go"], default="hg",
                   help="G1 / H1-2 / H2 = hg (default); legacy H1 = go.")
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--kd", type=float, default=8.0, help="Damping gain per motor.")
    p.add_argument("--rate", type=float, default=500.0, help="Damping command rate (Hz).")
    p.add_argument("--kill", default="unitree_setup.py",
                   help="pgrep -f pattern of controller processes to SIGINT on trigger; '' disables.")
    args = p.parse_args(argv)
    estop = EStop(args)
    estop.connect()
    return estop.run()


if __name__ == "__main__":
    sys.exit(main())
