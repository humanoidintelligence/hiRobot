"""Command-line entry point for hiRobot."""

from __future__ import annotations

import argparse
import sys

import yaml

from hirobot import __version__
from hirobot.loading.config import available_configs, load_config, resolve


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hirobot",
        description=(
            "Unified interface layer for controlling diverse robots from VLA models and agents."
        ),
    )
    parser.add_argument("--version", action="version", version=f"hirobot {__version__}")

    sub = parser.add_subparsers(dest="command")

    info = sub.add_parser("info", help="Show the embodiment described by a config.")
    info.add_argument("--config", required=True,
                      help="Built-in config name (see `hirobot configs`) or a file path.")
    info.add_argument("--resolved", action="store_true",
                      help="Print the whole config with its defaults: expanded, as YAML.")

    sim = sub.add_parser(
        "sim", help="Run a simulated robot: connect, hold its pose, report drift and images.")
    sim.add_argument("config", help="Built-in config name or file path (a simulated body).")
    sim.add_argument("--seconds", type=float, default=3.0, help="How long to hold (sim time).")
    sim.add_argument("--viewer", action="store_true", help="Open the MuJoCo viewer.")
    sim.add_argument("--fast", action="store_true",
                     help="Run as fast as possible instead of in real time.")

    view = sub.add_parser(
        "view", help="Show a real robot's pose in a MuJoCo viewer (opened read-only).")
    view.add_argument("config", help="Built-in config name or file path of the real robot.")
    view.add_argument("--model", default=None,
                      help="MuJoCo config to draw into (default: <config>_mujoco, built-in).")
    view.add_argument("--driver", nargs="*", default=[], metavar="KEY=VALUE",
                      help="Overrides for the real body's driver, e.g. network_interface=eth0.")
    view.add_argument("--hz", type=float, default=30.0, help="Frames per second.")
    view.add_argument("--seconds", type=float, default=None, help="Stop after this long.")
    view.add_argument("--no-window", action="store_true",
                      help="Mirror without opening a window (prints the frame count; for checks).")

    check = sub.add_parser(
        "check", help="Check a config: --sim builds its simulation-check world and reports it.")
    check.add_argument("config", help="Built-in config name or file path.")
    check.add_argument("--sim", action="store_true", required=True,
                       help="Build the world the simulation check uses (every body's model:).")
    check.add_argument("--against-real", action="store_true",
                       help="Also open the real robot read-only and compare its end-effector "
                            "positions with the model's at the same joint angles.")
    check.add_argument("--driver", nargs="*", default=[], metavar="PART:KEY=VALUE",
                       help="Overrides for a body's driver, e.g. body:network_interface=eth0.")
    check.add_argument("--tolerance", type=float, default=0.02,
                       help="End-effector position error (m) above which --against-real fails.")

    configs = sub.add_parser("configs", help="List the built-in configs.")
    configs.add_argument("group", nargs="?", default=None,
                         help="Only this group: robots, body, end_effector, camera, microphone, "
                              "speaker.")

    return parser


def _sim(args: argparse.Namespace) -> int:
    """Connect a simulated robot, hold its pose and report what it does."""
    import time

    import numpy as np

    from hirobot import Action, ControlMode, make_robot

    from hirobot.loading.build import only_body
    from hirobot.loading.tree import load_robot

    try:
        spec = load_robot(args.config)
        path, dev = only_body(spec, "hirobot sim")
    except ValueError as exc:
        print(f"hirobot sim: {exc}", file=sys.stderr)
        return 2
    driver = (dev.driver or {"type": "toy"})["type"]
    in_world = spec.sim is not None  # the shared MuJoCo simulation (sim:)
    if in_world:
        robot = make_robot(args.config, sim={"realtime": not args.fast,
                                             **({"viewer": True} if args.viewer else {})})
    elif args.viewer:
        print(f"--viewer needs the MuJoCo simulation (a sim: section); {args.config} has none "
              f"(body driver {driver!r})", file=sys.stderr)
        return 2
    else:
        robot = make_robot(args.config)
    if not robot.simulated:
        print(f"{args.config} drives a real robot (body driver {driver!r}); "
              "`hirobot sim` only runs simulated ones.", file=sys.stderr)
        return 2
    emb = robot.embodiment
    robot.connect()
    try:
        obs = robot.get_observation()
        base0 = None if obs.base_position is None else obs.base_position.copy()
        welded = in_world and path in ((spec.sim or {}).get("fixed_base") or [])
        base = ("fixed base" if not emb.floating_base
                else "floating base, welded in sim" if welded else "floating base")
        print(f"Robot: {emb.name} ({emb.num_dofs} dofs, {emb.control_hz} Hz, {base})")
        if obs.base_position is not None:
            print(f"  base start:    {np.round(obs.base_position, 3).tolist()}")
        print(f"  images:        {({k: list(v.shape) for k, v in obs.images.items()}) or 'none'}")
        # Hold the pose at connect with the configured gains; the error left
        # after it shows whether the gains carry the robot against gravity.
        q0 = obs.joint_pos.copy()
        robot.reset()
        hold = Action(mode=ControlMode.PD, joint_target=q0)
        render = []
        for _ in range(int(args.seconds * emb.control_hz)):
            robot.send_action(hold)
            t = time.perf_counter()
            obs = robot.get_observation()
            render.append(time.perf_counter() - t)
        err = np.abs(obs.joint_pos - q0)
        worst = int(np.argmax(err))
        print(f"  after {args.seconds:g} s holding the start pose:")
        print(f"    max joint error: {err[worst]:.4f} rad ({emb.joint_names[worst]})")
        if obs.base_position is not None:
            print(f"    base:            {np.round(obs.base_position, 3).tolist()}")
            if base0 is not None and base0[2] - obs.base_position[2] > 0.2:
                print("    the base dropped: it fell. Plain PD does not balance a free "
                      "humanoid; add a balance controller (layers:) or weld the base "
                      "(sim: fixed_base: [body]).")
        print(f"    get_observation: {np.mean(render) * 1e3:.1f} ms mean")
        if not np.all(np.isfinite(obs.joint_pos)):
            print("    joint state is not finite: the simulation blew up", file=sys.stderr)
            return 1
    finally:
        robot.disconnect()
    return 0


def _overrides(items: list[str]) -> dict:
    out = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not key:
            raise SystemExit(f"--driver takes KEY=VALUE, got {item!r}")
        out[key] = yaml.safe_load(value)
    return out


def _view(args: argparse.Namespace) -> int:
    """Open the real robot read-only and draw its pose in MuJoCo."""
    from hirobot.sim import view

    model_cfg = args.model or view.default_model(args.config)
    if model_cfg is None:
        print(f"no built-in MuJoCo config for {args.config}; pass --model <config>",
              file=sys.stderr)
        return 2
    try:
        real = view.open_read_only(args.config, _overrides(args.driver))
        model = view.open_model(model_cfg, window=not args.no_window)
    except ValueError as exc:
        print(f"hirobot view: {exc}", file=sys.stderr)
        return 2
    real.connect()
    try:
        model.connect()
        try:
            mirror = view.PoseMirror(real, model)
            print(f"Viewing {real.embodiment.name} (read-only) in {model_cfg}; "
                  f"{len(mirror.model_idx)} joints matched by name. Close the window or Ctrl-C.")
            if mirror.unmatched_real:
                print(f"  not in the model: {mirror.unmatched_real}")
            frames = view.run(real, model, hz=args.hz, seconds=args.seconds)
            if args.no_window:
                print(f"  {frames} frames")
        finally:
            model.disconnect()
    finally:
        real.disconnect()
    return 0


def _check(args: argparse.Namespace) -> int:
    """Build the simulation-check world (and compare with the real robot); 1 on a problem."""
    from hirobot.sim.check_sim import check_sim

    drivers: dict = {}
    for item in args.driver:
        part, sep, rest = item.partition(":")
        if not sep or not part:
            raise SystemExit(f"--driver takes PART:KEY=VALUE, got {item!r}")
        drivers.setdefault(part.strip("/"), {}).update(_overrides([rest]))
    try:
        report = check_sim(args.config, against_real=args.against_real, drivers=drivers)
    except (ValueError, FileNotFoundError, NotImplementedError) as exc:
        print(f"hirobot check: {exc}", file=sys.stderr)
        return 1
    print(f"Simulation check world of {args.config}: {len(report.bodies)} bodies, "
          f"{report.joints} joints")
    for path, mjcf in report.bodies.items():
        print(f"  {path}: {mjcf}")
    print(f"  end effectors with a frame: {report.end_effectors}")
    for w in report.warnings:
        print(f"  warning: {w}")
    status = 0
    if report.ee_errors is not None:
        if not report.ee_errors:
            print("  real robot: no end-effector poses reported; nothing to compare")
        for name, err in report.ee_errors.items():
            bad = err > args.tolerance
            status = 1 if bad else status
            print(f"  {name}: measured vs model {err * 1e3:.1f} mm"
                  f"{'  > ' + format(args.tolerance * 1e3, '.0f') + ' mm' if bad else ''}")
    print("OK" if status == 0 else "MISMATCH")
    return status


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "configs":
        for name in available_configs(args.group):
            print(name)
        return 0

    if args.command == "info":
        config = load_config(args.config)
        if args.resolved:
            print(f"# {resolve(args.config)}")
            print(yaml.safe_dump(config, sort_keys=False, default_flow_style=None), end="")
            return 0
        from hirobot import load_embodiment

        embodiment = load_embodiment(args.config)
        print(f"Embodiment: {embodiment.name}")
        print(f"  kind:          {embodiment.kind.value}")
        print(f"  control_hz:    {embodiment.control_hz}")
        print(f"  floating_base: {embodiment.floating_base}")
        print(f"  dofs ({embodiment.num_dofs}):      {embodiment.joint_names}")
        print(f"  end_effectors: {[e.name for e in embodiment.end_effectors]}")
        print(f"  cameras:       {[c.name for c in embodiment.cameras]}")
        print(f"  microphones:   {[m.name for m in embodiment.microphones]}")
        print(f"  speakers:      {[s.name for s in embodiment.speakers]}")
        parts = [p for p in embodiment.parts if p]
        if parts:
            print(f"  parts:         {parts}")
        return 0

    if args.command == "sim":
        return _sim(args)

    if args.command == "view":
        return _view(args)

    if args.command == "check":
        return _check(args)

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
