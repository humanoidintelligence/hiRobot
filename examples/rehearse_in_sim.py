"""Rehearse a motion in MuJoCo, starting from where the real robot stands.

The real robot is opened read-only (no command publisher is created), its
current pose is read once, and the same motion runs on the robot's
``*_mujoco`` config started from that pose. Nothing is sent to the real robot.
When the rehearsal looks right, run the same code on the real config.

    python examples/rehearse_in_sim.py unitree_g1 unitree_g1_mujoco \\
        --driver network_interface=eth0 --joint right_elbow --target 0.0

``read_real_pose`` and ``sim_from_pose`` are the two pieces to copy; the
motion in ``rehearse`` stands in for your own.
"""

from __future__ import annotations

import argparse

import numpy as np

from hirobot import Action, ControlMode, Robot, make_robot


def read_real_pose(real_config, driver=None):
    """Connect read-only, read one observation, disconnect.

    ``driver`` is merged over the body's driver (``{"network_interface": "eth0"}``).
    """
    real = make_robot(real_config,
                      overrides={"body": {"driver": dict(driver or {}, read_only=True)}})
    real.connect()
    try:
        return real.get_observation()
    finally:
        real.disconnect()


def sim_from_pose(sim_config, obs, sim=None) -> Robot:
    """Build the MuJoCo robot and put it in the measured pose.

    ``sim`` is merged over the config's ``sim:`` section (``{"realtime": False}``).
    """
    from hirobot.sim.view import sim_view

    sim = make_robot(sim_config, sim=dict(sim) if sim else True)
    sim.connect()
    # set_state is on the MuJoCo view, under the gate and the assembly.
    sim_view(sim).set_state(obs.joint_pos, obs.joint_vel, obs.base_orientation)
    sim.reset()  # ramps the gains up while holding that pose
    return sim


def rehearse(sim: Robot, joint: str, target: float, seconds: float) -> np.ndarray:
    """Ramp one joint to ``target`` in ``seconds``; return the final joint positions."""
    emb = sim.embodiment
    j = emb.joint_names.index(joint)
    start = sim.get_observation().joint_pos.copy()
    steps = max(1, int(seconds * emb.control_hz))
    for k in range(1, steps + 1):
        q = start.copy()
        q[j] = start[j] + (target - start[j]) * k / steps
        sim.send_action(Action(mode=ControlMode.PD, joint_target=q))
    return sim.get_observation().joint_pos


def _kv(items):
    out = {}
    for item in items or []:
        key, _, value = item.partition("=")
        out[key] = value
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("real", help="config of the real robot (opened read-only)")
    ap.add_argument("sim", help="its MuJoCo config, e.g. unitree_g1_mujoco")
    ap.add_argument("--driver", nargs="*",
                    help="key=value overrides for the real body's driver (network_interface=...)")
    ap.add_argument("--joint", default="right_elbow")
    ap.add_argument("--target", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args(argv)

    obs = read_real_pose(args.real, _kv(args.driver))
    sim = sim_from_pose(args.sim, obs)
    try:
        q = rehearse(sim, args.joint, args.target, args.seconds)
    finally:
        sim.disconnect()
    j = sim.embodiment.joint_names.index(args.joint)
    print(f"{args.joint}: real {obs.joint_pos[j]:+.3f} -> sim {q[j]:+.3f} (target {args.target:+.3f})")


if __name__ == "__main__":
    main()
