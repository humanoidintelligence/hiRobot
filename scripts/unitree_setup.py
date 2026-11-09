"""Unitree real-robot setup checks.

Two steps, run in order the first time a robot is connected:

``status`` (read-only, publishes nothing)
    Connects, reads ``rt/lowstate`` for a few seconds and prints the state rate,
    ``mode_machine``, IMU, per-joint position / torque / temperature, and which
    built-in motion service (if any) owns ``rt/lowcmd``.

``hold`` (commands the robot; hang it first, e-stop in reach)
    Optionally releases the built-in motion service, then runs ``reset()``: it
    reads the measured pose and ramps kp from 0 over ``ramp_seconds`` while
    holding that pose. It keeps holding for ``--seconds`` and aborts to damping
    if any joint drifts more than ``--max-error`` rad. On exit (normal, error,
    or Ctrl-C) the robot is left in damping mode.

``move`` (commands the robot; hang it first, e-stop in reach)
    Ramps gains to hold the measured pose, moves one joint to an absolute
    target on a minimum-jerk trajectory, holds, moves back, then damps. Aborts
    if the joint lags its trajectory or any other joint drifts.

``--sim off|mirror|shadow`` (``hold`` and ``move``)
    ``off`` drives the real robot only and needs no MuJoCo. ``mirror`` drives
    the real robot and renders its measured state in MuJoCo. ``shadow`` opens
    the real robot read-only and runs the same motion on the MuJoCo twin from
    the robot's current pose. Needs a ``sim:`` section (or ``--sim-model``).

``restore`` (hands control back to the built-in service)
    Calls the motion switcher's ``SelectMode`` (default ``ai``), waits until it
    reports active, then by default asks it to enter its damping state and
    watches the motors for a few seconds.

Host-specific values come from the command line, not the config::

    python scripts/unitree_setup.py status --config unitree_g1 --iface eth0
    python scripts/unitree_setup.py hold --config my_g1_real.yaml --iface eth0 \
        --release-motion-mode --seconds 10

The config's ``body:`` section may set ``driver: toy``; this script always uses
the ``unitree`` driver and passes ``--iface`` / ``--msg-family`` through.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from hirobot import Action, ControlMode, make_robot


def _slots(robot, n: int) -> list:
    """SDK motor slot of each joint (the legacy H1 skips slot 9)."""
    return list(getattr(robot, "motor_index", None) or range(n))


def _enabled(s, slots) -> int:
    """Motors reporting a nonzero mode (hg: 1; legacy H1: 0x01 / 0x0A)."""
    return sum(1 for i in slots if s.motor_state[i].mode != 0)


def build_robot(args, release: bool):
    sim = getattr(args, "sim", None)
    if sim == "shadow":
        release = False  # shadow never commands the real robot
    overrides = {"driver": "unitree_sdk2", "network_interface": args.iface, "release_motion_mode": release}
    if getattr(args, "ramp", None) is not None:
        overrides["ramp_seconds"] = args.ramp
    if args.msg_family:
        overrides["msg_family"] = args.msg_family
    sim_overrides = {}
    if sim is not None:
        sim_overrides["mode"] = sim
    if getattr(args, "sim_model", None):
        sim_overrides["model"] = args.sim_model
    return make_robot(args.config, body_overrides=overrides, sim_overrides=sim_overrides)


def _check_motion_service(robot, args) -> bool:
    """True if nothing contests rt/lowcmd (always true in shadow mode)."""
    if getattr(robot, "mode", None) == "shadow" or args.release_motion_mode:
        return True
    real = getattr(robot, "real", robot)
    mode = real.motion_mode()
    if mode and mode.get("name"):
        print(f"Built-in motion service {mode['name']!r} is active; it would fight our commands.")
        print("Re-run with --release-motion-mode (robot hung!), or use --sim shadow to rehearse in sim.")
        return False
    return True


def _final_state(robot) -> str:
    if getattr(robot, "mode", None) == "shadow":
        return "Sim twin left in damping mode; the real robot was never commanded."
    return "Robot left in damping mode."


def _describe_target(robot) -> None:
    mode = getattr(robot, "mode", "off")
    if mode == "shadow":
        print("[sim shadow] real robot is read-only; commands go to the MuJoCo twin only.")
    elif mode == "mirror":
        print("[sim mirror] commanding the REAL robot; MuJoCo shows its measured state.")


def cmd_status(args) -> int:
    robot = build_robot(args, release=False)
    robot.connect()
    names = robot.embodiment.joint_names
    count, t0, last_tick = 0, time.time(), None
    while time.time() - t0 < args.seconds:
        tick = robot.raw_state.tick
        if tick != last_tick:
            count, last_tick = count + 1, tick
        time.sleep(0.0005)
    s = robot.raw_state
    obs = robot.get_observation()

    print(f"lowstate: ~{count / args.seconds:.0f} Hz (sampled)  tick={s.tick}")
    if hasattr(s, "mode_machine"):
        print(f"mode_machine={s.mode_machine}  mode_pr={s.mode_pr}")
    imu = s.imu_state
    print(f"IMU rpy(deg)={np.round(np.degrees(imu.rpy), 1)}  acc={np.round(imu.accelerometer, 2)}")
    print(f"{'joint':<24}{'q(rad)':>9}{'tau':>8}{'temp(C)':>10}{'mode':>6}")
    temps = []
    slots = _slots(robot, len(names))
    for i, name in enumerate(names):
        m = s.motor_state[slots[i]]
        t = max(m.temperature) if hasattr(m.temperature, "__len__") else m.temperature
        temps.append(t)
        print(f"{name:<24}{obs.joint_pos[i]:>9.3f}{obs.joint_torque[i]:>8.2f}{t:>10}{m.mode:>6}")
    hot = int(np.argmax(temps))
    print(f"max motor temp: {temps[hot]} C ({names[hot]})")
    try:
        print(f"built-in motion service: {robot.motion_mode()}")
    except Exception as exc:  # noqa: BLE001 - switcher is optional on some firmware
        print(f"built-in motion service: query failed ({exc})")
    return 0


def cmd_hold(args) -> int:
    robot = build_robot(args, release=args.release_motion_mode)
    robot.connect()  # blocks while releasing the motion service, if asked
    _describe_target(robot)
    if not _check_motion_service(robot, args):
        robot.disconnect()
        return 2
    names = robot.embodiment.joint_names
    watch = _watch_indices(names, args.torque_joints)
    print_torque = _torque_printer(robot, names, watch)
    mon = None
    try:
        start = robot.get_observation().joint_pos.copy()
        print_torque("before ramp", header=True)
        print(f"Ramping gains to hold the measured pose over {args.ramp:.1f}s ...")
        robot.reset()
        print_torque("after ramp")
        hold_pose = Action(mode=ControlMode.PD, joint_target=start)  # driver's configured kp/kd
        low_cmd = getattr(robot, "_low_cmd", None)
        if low_cmd is not None:
            kp_cmd = np.array([low_cmd.motor_cmd[i].kp for i in _slots(robot, len(names))])
            print(f"commanded kp after ramp: min {kp_cmd.min():.0f}  max {kp_cmd.max():.0f}")
        mon = _HoldMonitor(robot, names, watch, start, args.log)
        mon.header()
        period = 1.0 / robot.embodiment.control_hz
        t0 = time.time()
        t_end = t0 + args.seconds
        next_print = t0 + 1.0
        while time.time() < t_end:
            robot.send_action(hold_pose)  # goes through the safety gate
            err = mon.sample(time.time() - t0)
            if err.max() > args.max_error:
                j = names[int(err.argmax())]
                print(f"ABORT: {j} drifted {err.max():.3f} rad (> {args.max_error}); damping.")
                return 3
            if time.time() >= next_print:
                mon.flush(f"{time.time() - t0:4.0f}s")
                next_print += 1.0
            time.sleep(period)
        print(f"Held for {args.seconds:.1f}s; worst tracking error {mon.worst:.3f} rad "
              f"({names[mon.worst_joint]}); peak |tau| {mon.peak_tau:.2f} Nm ({names[mon.peak_joint]}).")
        return 0
    finally:
        robot.disconnect()  # sends damping
        if mon is not None:
            mon.close()
        time.sleep(0.5)
        print_torque("damped +0.5s")
        print(_final_state(robot))


def _min_jerk(s: float) -> float:
    """Minimum-jerk profile on [0, 1]: zero velocity and acceleration at both ends."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def cmd_move(args) -> int:
    """Move one joint to an absolute target and back while the rest hold their pose."""
    robot = build_robot(args, release=args.release_motion_mode)
    names = robot.embodiment.joint_names
    if args.joint not in names:
        print(f"Unknown joint {args.joint!r}; choose from: {', '.join(names)}")
        return 2
    robot.connect()
    _describe_target(robot)
    if not _check_motion_service(robot, args):
        robot.disconnect()
        return 2
    j = names.index(args.joint)
    log = open(args.log, "w") if args.log else None  # noqa: SIM115 - closed in finally
    try:
        start = robot.get_observation().joint_pos.copy()
        travel = args.target - start[j]
        print(f"{args.joint}: measured {start[j]:.3f} rad -> target {args.target:.3f} rad "
              f"(travel {travel:+.3f} rad = {np.degrees(travel):+.1f} deg over {args.duration:.1f}s, "
              f"peak speed {1.875 * abs(travel) / args.duration:.2f} rad/s)")
        if abs(travel) > args.max_travel:
            print(f"Refusing: travel exceeds --max-travel {args.max_travel} rad.")
            return 2
        print(f"Ramping gains to hold the measured pose over {args.ramp:.1f}s ...")
        robot.reset()
        if log:
            log.write("t,phase,desired,measured,tau,max_other_dev\n")
        others = np.ones(len(names), dtype=bool)
        others[j] = False

        # (phase, duration, from, to)
        segments = [("out", args.duration, start[j], args.target), ("hold", args.hold, args.target, args.target)]
        if not args.no_return:
            segments.append(("back", args.duration, args.target, start[j]))
            segments.append(("settle", 1.0, start[j], start[j]))
        print(f"{'t':>6} {'phase':<7}{'desired':>9}{'measured':>10}{'track err':>11}{'tau':>8}"
              f"{'max other dev':>24}")
        period = 1.0 / robot.embodiment.control_hz
        t0 = time.time()
        worst_track, worst_other, peak_tau = 0.0, 0.0, 0.0
        for phase, dur, a, b in segments:
            ts = time.time()
            next_print = ts
            while True:
                now = time.time()
                s = (now - ts) / dur if dur > 0 else 1.0
                desired = a + (b - a) * _min_jerk(s)
                target = start.copy()
                target[j] = desired
                robot.send_action(Action(mode=ControlMode.PD, joint_target=target))  # safety gate
                obs = robot.get_observation()
                q = obs.joint_pos
                tau = obs.joint_torque if obs.joint_torque is not None else np.zeros(len(names))
                track = abs(q[j] - desired)
                dev = np.where(others, np.abs(q - start), 0.0)
                k = int(dev.argmax())
                worst_track = max(worst_track, track)
                worst_other = max(worst_other, float(dev[k]))
                peak_tau = max(peak_tau, abs(float(tau[j])))
                if log:
                    log.write(f"{now - t0:.3f},{phase},{desired:.4f},{q[j]:.4f},{tau[j]:.3f},{dev[k]:.4f}\n")
                if track > args.max_track_error:
                    print(f"ABORT: {args.joint} tracking error {track:.3f} rad (> {args.max_track_error}); damping.")
                    return 3
                if dev[k] > args.max_error:
                    print(f"ABORT: {names[k]} drifted {dev[k]:.3f} rad (> {args.max_error}); damping.")
                    return 3
                if now >= next_print:
                    print(f"{now - t0:6.2f} {phase:<7}{desired:>9.3f}{q[j]:>10.3f}{track:>11.3f}{tau[j]:>8.2f}"
                          f"{dev[k]:>10.3f} {_short(names[k]):<13}")
                    next_print += 0.5
                if s >= 1.0:
                    break
                time.sleep(period)
        print(f"Done. worst tracking error {worst_track:.3f} rad; worst drift of other joints "
              f"{worst_other:.3f} rad; peak |tau| on {args.joint} {peak_tau:.2f} Nm.")
        return 0
    finally:
        robot.disconnect()  # sends damping
        if log:
            log.close()
        print(_final_state(robot))


class _HoldMonitor:
    """Sample every control tick; print per-second peaks so short pushes are not missed.

    Each printed row covers the last second: the peak |tau_est| of the watched
    joints, the joint with the largest deviation from the hold pose and its peak,
    how many motors report mode == 1 (enabled), and how many new lowstate ticks
    arrived (0 means the state stream stalled). ``log_path`` optionally writes
    every sample (t, q error, tau, motor mode) as CSV.
    """

    def __init__(self, robot, names, watch, start, log_path=None) -> None:
        self.robot, self.names, self.watch, self.start = robot, names, watch, start
        self.n = len(names)
        self.worst, self.worst_joint = 0.0, 0
        self.peak_tau, self.peak_joint = 0.0, 0
        self._reset_window()
        state = getattr(robot, "raw_state", None)
        self._last_tick = None if state is None else state.tick
        self._log = None
        if log_path:
            self._log = open(log_path, "w")  # noqa: SIM115 - closed in close()
            cols = ["t"] + [f"err_{n}" for n in names] + [f"tau_{n}" for n in names] + ["enabled"]
            self._log.write(",".join(cols) + "\n")
        short = [_short(names[i]) for i in watch]
        self._width = max([len(s) for s in short] + [6]) + 1
        self._short = short

    def _reset_window(self) -> None:
        self._tau_peak = np.zeros(self.n)
        self._err_peak = np.zeros(self.n)
        self._enabled_min = self.n
        self._ticks = 0

    def header(self) -> None:
        w = self._width
        print("peak per 1s window   |tau_est| (Nm)")
        print(f"{'t':<6}" + "".join(f"{s:>{w}}" for s in self._short)
              + f"{'max dev (rad)':>30}{'enabled':>9}{'ticks':>7}")

    def sample(self, t: float) -> np.ndarray:
        obs = self.robot.get_observation()
        s = getattr(self.robot, "raw_state", None)  # None for the sim twin
        err = np.abs(obs.joint_pos - self.start)
        tau = np.abs(obs.joint_torque)
        enabled = self.n if s is None else _enabled(s, _slots(self.robot, self.n))
        np.maximum(self._tau_peak, tau, out=self._tau_peak)
        np.maximum(self._err_peak, err, out=self._err_peak)
        self._enabled_min = min(self._enabled_min, enabled)
        tick = None if s is None else s.tick
        if s is None or tick != self._last_tick:
            self._ticks += 1  # counts control ticks that saw fresh state
            self._last_tick = tick
        k = int(err.argmax())
        if err[k] > self.worst:
            self.worst, self.worst_joint = float(err[k]), k
        k = int(tau.argmax())
        if tau[k] > self.peak_tau:
            self.peak_tau, self.peak_joint = float(tau[k]), k
        if self._log:
            row = [f"{t:.3f}"] + [f"{v:.4f}" for v in err] + [f"{v:.3f}" for v in obs.joint_torque] + [str(enabled)]
            self._log.write(",".join(row) + "\n")
        return err

    def flush(self, label: str) -> None:
        w = self._width
        k = int(self._err_peak.argmax())
        dev = f"{self._err_peak[k]:.3f} {_short(self.names[k])}"
        print(f"{label:<6}" + "".join(f"{self._tau_peak[i]:>{w}.2f}" for i in self.watch)
              + f"{dev:>30}{self._enabled_min:>5}/{self.n:<3}{self._ticks:>7}")
        self._reset_window()

    def close(self) -> None:
        if self._log:
            self._log.close()


def _short(name: str) -> str:
    return (name.replace("shoulder", "sh").replace("_pitch", "_p").replace("left_", "L_")
            .replace("right_", "R_").replace("waist", "w"))


def _watch_indices(names, spec: str):
    """Joints whose name contains any comma-separated substring in ``spec``."""
    keys = [k.strip() for k in spec.split(",") if k.strip()]
    return [i for i, n in enumerate(names) if any(k in n for k in keys)]


def _torque_printer(robot, names, watch):
    """Return a function printing estimated torque (Nm) of the watched joints."""
    short = [_short(names[i]) for i in watch]
    width = max([len(s) for s in short] + [6]) + 1

    def emit(label: str, header: bool = False) -> None:
        tau = robot.get_observation().joint_torque
        if header:
            print(f"{'tau_est (Nm)':<14}" + "".join(f"{s:>{width}}" for s in short) + "   max|tau| (joint)")
        if tau is None:
            print(f"{label:<14}(driver reports no joint torque)")
            return
        k = int(np.argmax(np.abs(tau)))
        s = getattr(robot, "raw_state", None)
        enabled = "" if s is None else (
            f"   enabled {_enabled(s, _slots(robot, len(names)))}/{len(names)}")
        print(f"{label:<14}" + "".join(f"{tau[i]:>{width}.2f}" for i in watch)
              + f"   {abs(tau[k]):.2f} ({names[k]}){enabled}")

    return emit


def _loco_fsm(loco) -> str:
    """Read the built-in controller's FSM id (API 7001); the SDK has no wrapper for it."""
    try:
        code, data = loco._Call(7001, "{}")
        return data if code == 0 else f"unavailable (code {code})"
    except Exception as exc:  # noqa: BLE001 - diagnostic only
        return f"unavailable ({exc})"


def cmd_restore(args) -> int:
    """Hand ``rt/lowcmd`` back to a built-in motion service via the motion switcher."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    if args.msg_family == "go":
        from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
    else:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    robot = build_robot(args, release=False)
    robot.connect()  # subscribes to lowstate; publishes nothing
    names = robot.embodiment.joint_names
    n = len(names)

    def enabled() -> int:
        s = robot.raw_state
        return _enabled(s, _slots(robot, n))

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, mode = msc.CheckMode()
    print(f"before: CheckMode code={code} mode={mode}  motors enabled {enabled()}/{n}")
    if code == 0 and mode and mode.get("name"):
        print(f"{mode['name']!r} is already active; nothing to do.")
        return 0

    print(f"SelectMode({args.service!r}) ...")
    code, _ = msc.SelectMode(args.service)
    print(f"  SelectMode returned code={code}")
    if code != 0:
        print("Motion switcher refused the request; the robot was not changed.")
        return 2

    t0 = time.time()
    active = None
    while time.time() - t0 < args.timeout:
        c, m = msc.CheckMode()
        if c == 0 and m and m.get("name"):
            active = m
            break
        time.sleep(0.5)
    if active is None:
        print(f"  service did not report active within {args.timeout:.0f}s")
        return 3
    print(f"  active after {time.time() - t0:.1f}s: {active}")

    loco = LocoClient()
    loco.SetTimeout(3.0)
    loco.Init()
    print(f"  loco FSM: {_loco_fsm(loco)}  motors enabled {enabled()}/{n}")
    if args.then == "damp":
        loco.Damp()
        time.sleep(1.0)
        print(f"  sent loco Damp(); FSM now: {_loco_fsm(loco)}")

    # Watch what the service does to the motors for a few seconds.
    start = robot.get_observation().joint_pos.copy()
    peak_dev = np.zeros(n)
    peak_tau = np.zeros(n)
    t0 = time.time()
    while time.time() - t0 < args.watch:
        obs = robot.get_observation()
        np.maximum(peak_dev, np.abs(obs.joint_pos - start), out=peak_dev)
        np.maximum(peak_tau, np.abs(obs.joint_torque), out=peak_tau)
        time.sleep(0.02)
    kd, kt = int(peak_dev.argmax()), int(peak_tau.argmax())
    print(f"watched {args.watch:.0f}s: motors enabled {enabled()}/{n}; "
          f"max dev {peak_dev[kd]:.3f} rad ({names[kd]}); peak |tau| {peak_tau[kt]:.2f} Nm ({names[kt]})")
    print("Built-in control restored. Use the remote for further state changes.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("status", "hold", "move", "restore"):
        sp = sub.add_parser(name)
        sp.add_argument("--config", required=True, help="Robot config (embodiment + robot section).")
        sp.add_argument("--iface", required=True, help="Host NIC on the robot subnet, e.g. eth0.")
        sp.add_argument("--msg-family", choices=["hg", "go"], default=None,
                        help="Override robot.msg_family (G1/H1-2/H2 = hg, legacy H1 = go).")
    sub.choices["status"].add_argument("--seconds", type=float, default=2.0)
    hold = sub.choices["hold"]
    hold.add_argument("--seconds", type=float, default=5.0, help="How long to hold after the ramp.")
    hold.add_argument("--ramp", type=float, default=2.0, help="kp ramp duration (s).")
    hold.add_argument("--max-error", type=float, default=0.3, help="Abort if any joint drifts this far (rad).")
    hold.add_argument("--torque-joints", default="hip_pitch,knee,waist_pitch,shoulder_pitch,elbow",
                      help="Comma-separated name substrings of joints whose torque is printed.")
    hold.add_argument("--log", default=None, help="Optional CSV path: every sample's error, torque, enabled count.")
    hold.add_argument("--release-motion-mode", action="store_true",
                      help="Release the built-in motion service first. The robot loses balance: hang it.")
    for name in ("hold", "move"):
        sp = sub.choices[name]
        sp.add_argument("--sim", choices=["off", "mirror", "shadow"], default=None,
                        help="MuJoCo twin: off = real robot only (no MuJoCo needed); mirror = command "
                        "the real robot and render its state; shadow = real robot read-only, "
                        "rehearse the motion in sim. Default: the config's sim.mode (off if absent).")
        sp.add_argument("--sim-model", default=None, help="Override sim.model (MJCF path).")
    move = sub.choices["move"]
    move.add_argument("--joint", required=True, help="Joint name, e.g. right_elbow.")
    move.add_argument("--target", type=float, required=True, help="Absolute target position (rad).")
    move.add_argument("--duration", type=float, default=4.0, help="Seconds for the move out (and back).")
    move.add_argument("--hold", type=float, default=2.0, help="Seconds to hold at the target.")
    move.add_argument("--no-return", action="store_true", help="Damp at the target instead of moving back.")
    move.add_argument("--ramp", type=float, default=2.0, help="kp ramp duration (s).")
    move.add_argument("--max-travel", type=float, default=1.6, help="Refuse moves longer than this (rad).")
    move.add_argument("--max-track-error", type=float, default=0.3,
                      help="Abort if the moving joint lags its trajectory by this much (rad).")
    move.add_argument("--max-error", type=float, default=0.3, help="Abort if any other joint drifts this far (rad).")
    move.add_argument("--log", default=None, help="Optional CSV path for every sample.")
    move.add_argument("--release-motion-mode", action="store_true",
                      help="Release the built-in motion service first. The robot loses balance: hang it.")
    restore = sub.choices["restore"]
    restore.add_argument("--service", default="ai", help="Motion service to select (G1: ai / normal / advanced).")
    restore.add_argument("--then", choices=["damp", "none"], default="damp",
                         help="After the service is up, ask it to enter its damping state (default).")
    restore.add_argument("--timeout", type=float, default=15.0, help="Wait this long for the service to come up.")
    restore.add_argument("--watch", type=float, default=3.0, help="Then watch the motors for this long (s).")
    args = p.parse_args(argv)
    return {"status": cmd_status, "hold": cmd_hold, "move": cmd_move, "restore": cmd_restore}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
