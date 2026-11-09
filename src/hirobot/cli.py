"""Command-line entry point for hiRobot."""

from __future__ import annotations

import argparse
import sys

import yaml

from hirobot import __version__
from hirobot.config import available_configs, load_config, resolve
from hirobot.embodiment import Camera, Embodiment


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

    configs = sub.add_parser("configs", help="List the built-in configs.")
    configs.add_argument("group", nargs="?", default=None,
                         help="Only this group: robots, body, end_effector, camera, audio, sim, groups.")

    return parser


def _group_info(ref: str, config: dict) -> int:
    """Print a robot group's members (their configs, not connecting anything)."""
    from hirobot.group import group_embodiment

    g = config["group"] or {}
    base = resolve(ref).parent
    members = {}
    for name, spec in (g.get("members") or {}).items():
        member_cfg = load_config(resolve(spec["config"], base=base) if isinstance(spec["config"], str)
                                 else spec["config"])
        members[name] = Embodiment(**member_cfg["embodiment"])
        layers = [entry.get("type") for entry in member_cfg.get("layers") or []]
        print(f"Member {name}: {spec['config']} ({members[name].name}, "
              f"{members[name].num_dofs} dofs"
              f"{', base_pose ' + str(list(spec['base_pose'])) if spec.get('base_pose') else ''}"
              f"{', layers: ' + ', '.join(layers) if layers else ''})")
    emb = group_embodiment(g.get("name", "robot_group"), members,
                           [Camera(**c) for c in g.get("cameras") or []], g.get("control_hz"),
                           g.get("expose"))
    print(f"Group: {emb.name}")
    print(f"  control_hz:    {emb.control_hz}")
    print(f"  floating_base: {emb.floating_base}")
    print(f"  dofs ({emb.num_dofs}):      {emb.joint_names}")
    print(f"  end_effectors: {[e.name for e in emb.end_effectors]}")
    print(f"  cameras:       {[c.name for c in emb.cameras]}")
    return 0


def _sim(args: argparse.Namespace) -> int:
    """Connect a simulated robot, hold its pose and report what it does."""
    import time

    import numpy as np

    from hirobot import Action, ControlMode, make_robot

    driver = (load_config(args.config).get("body") or {}).get("driver")
    over = {}
    if driver == "mujoco":  # realtime / viewer are MuJoCo options
        over["realtime"] = not args.fast
        if args.viewer:
            over["viewer"] = True
    elif args.viewer:
        print(f"--viewer needs a MuJoCo body; {args.config} uses {driver!r}", file=sys.stderr)
        return 2
    robot = make_robot(args.config, body_overrides=over)
    if not robot.simulated:
        print(f"{args.config} drives a real robot (body.driver: {type(robot).__name__}); "
              "`hirobot sim` only runs simulated ones.", file=sys.stderr)
        return 2
    emb = robot.embodiment
    robot.connect()
    try:
        obs = robot.get_observation()
        base0 = None if obs.base_position is None else obs.base_position.copy()
        welded = bool((load_config(args.config).get("body") or {}).get("fixed_base"))
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
                      "(fixed_base: true).")
        print(f"    get_observation: {np.mean(render) * 1e3:.1f} ms mean")
        if not np.all(np.isfinite(obs.joint_pos)):
            print("    joint state is not finite: the simulation blew up", file=sys.stderr)
            return 1
    finally:
        robot.disconnect()
    return 0


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
        if "group" in config:
            return _group_info(args.config, config)
        embodiment = Embodiment(**config["embodiment"])
        print(f"Embodiment: {embodiment.name}")
        print(f"  kind:          {embodiment.kind.value}")
        print(f"  control_hz:    {embodiment.control_hz}")
        print(f"  floating_base: {embodiment.floating_base}")
        print(f"  dofs ({embodiment.num_dofs}):      {embodiment.joint_names}")
        print(f"  end_effectors: {[e.name for e in embodiment.end_effectors]}")
        print(f"  cameras:       {[c.name for c in embodiment.cameras]}")
        print(f"  microphones:   {[m.name for m in embodiment.microphones]}")
        print(f"  speakers:      {[s.name for s in embodiment.speakers]}")
        return 0

    if args.command == "sim":
        return _sim(args)

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
