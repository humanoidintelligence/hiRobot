"""Move the G1's right hand in MuJoCo with the mink_ik layer (CARTESIAN_DELTA).

The G1 hangs on a gantry (unitree_g1_mujoco: pelvis welded in place). The
mink_ik layer goes on top through ``layers:``; it takes per-tick deltas for the
hands and sends joint POSITION targets to the simulated robot. The right hand
moves 5 cm forward, then 5 cm up (1 s each at 50 Hz); the left hand is not
named, so it keeps its pose.

    pip install 'hirobot[sim,ik]'      # mink needs Python >= 3.11
    python examples/mink_ik_g1.py [--viewer]

The G1 MJCF is not shipped; docs/setup/unitree.md says how to fetch it.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from hirobot import Action, ControlMode, make_robot
from hirobot.loading.config import load_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--viewer", action="store_true", help="open the MuJoCo window")
    args = parser.parse_args()

    # Without model:, the layer uses the MuJoCo model of the simulated robot below.
    # frames: names the MJCF bodies; left out, it takes the end-effectors' frames
    # from the robot config (the same two links for the G1).
    layer = {"type": "mink_ik", "frames": {"left_hand": {"name": "left_wrist_yaw_link"},
                                           "right_hand": {"name": "right_wrist_yaw_link"}}}
    # In a YAML file this is `defaults: [unitree_g1_mujoco]` plus a `layers:` list.
    config = dict(load_config("unitree_g1_mujoco"), layers=[layer])
    robot = make_robot(config, sim={"realtime": args.viewer, "viewer": args.viewer})
    print("modes:", [m.value for m in robot.action_modes], "| ik frames:", robot.cartesian_targets)
    robot.connect()
    try:
        robot.reset()
        hz = robot.embodiment.control_hz
        # 1 s of empty deltas first: the arms settle under gravity at the pose
        # the IK took at the first command.
        for _ in range(hz):
            robot.send_action(Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}))
        start = robot.get_observation().ee_poses
        # Where the commanded joints put the hands (forward kinematics of the
        # last joint target). The measured pose is a little lower: the arms
        # track it on PD and sag under gravity.
        start_cmd = robot.commanded_ee_poses
        steps = hz  # 1 s per leg
        legs = [np.array([0.05, 0.0, 0.0]), np.array([0.0, 0.0, 0.05])]  # +x then +z, metres
        t0 = time.perf_counter()
        for leg in legs:
            delta = np.concatenate([leg / steps, np.zeros(3)])
            for _ in range(steps):
                robot.send_action(Action(mode=ControlMode.CARTESIAN_DELTA,
                                         ee_targets={"right_hand": delta}))
        wall = time.perf_counter() - t0
        for _ in range(hz // 2):  # 0.5 s more with no new delta: let the joints settle
            robot.send_action(Action(mode=ControlMode.CARTESIAN_DELTA, ee_targets={}))
        end = robot.get_observation().ee_poses
        end_cmd = robot.commanded_ee_poses
        want = sum(legs)
        cm = 100.0

        def show(label, a, b):
            moved = b["right_hand"][:3] - a["right_hand"][:3]
            print(f"{label}: start {np.round(a['right_hand'][:3], 4)} end "
                  f"{np.round(b['right_hand'][:3], 4)} moved {np.round(moved * cm, 2)} cm, "
                  f"error {np.linalg.norm(moved - want) * cm:.2f} cm")

        print(f"right hand, asked to move {np.round(want * cm, 1)} cm (base frame, m):")
        show("  measured ", start, end)
        show("  commanded", start_cmd, end_cmd)
        sag = np.linalg.norm(end["right_hand"][:3] - end_cmd["right_hand"][:3])
        print(f"  measured vs commanded at the end (PD sag): {sag * cm:.2f} cm")
        drift = np.linalg.norm(end["left_hand"][:3] - start["left_hand"][:3])
        print(f"left hand drift: {drift * cm:.2f} cm (not commanded)")
        print(f"{2 * steps} commands in {wall:.2f} s wall time ({wall / (2 * steps) * 1e3:.2f} ms "
              f"each, IK and physics); IK alone {robot.stats['mean_ms']:.2f} ms mean, "
              f"{robot.stats['max_ms']:.2f} ms max")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
