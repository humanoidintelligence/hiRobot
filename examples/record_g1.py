"""Record a few seconds of the MuJoCo G1 as a LeRobot v3.0 dataset, then read it back.

The G1 hangs on a gantry (unitree_g1_mujoco: pelvis welded in place) and waves
its right shoulder at 50 Hz; a :class:`~hirobot.record.Recorder` listens and,
after ``stop()``, saves the episode in the background: aligned to a 30 fps
grid, the head camera as H.264. The script then reads the files back and times
the control loop with and without recording.

    pip install 'hirobot[sim,record]'
    python examples/record_g1.py [--out DIR] [--seconds 4]

The G1 MJCF is not shipped; docs/setup/unitree.md says how to fetch it.
"""

from __future__ import annotations

import argparse
import json
import tempfile
import time
from pathlib import Path

import numpy as np

from hirobot import Action, ControlMode, make_robot
from hirobot.record import Recorder


def wave(robot, steps, pose, joint, times=None):
    """Swing ``joint`` around ``pose`` for ``steps`` ticks; optionally time each tick.

    The loop is paced at the robot's control rate, as a real one is: unpaced,
    a simulation runs many times faster than real time and its cameras produce
    frames faster than the recorder's save worker encodes them.
    """
    hz = robot.embodiment.control_hz
    next_tick = time.perf_counter()
    for k in range(steps):
        t0 = time.perf_counter()
        robot.get_observation()
        target = pose.copy()
        target[joint] += 0.4 * np.sin(2 * np.pi * 0.5 * k / hz)
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
        if times is not None:
            times.append(time.perf_counter() - t0)
        next_tick += 1.0 / hz
        time.sleep(max(0.0, next_tick - time.perf_counter()))


#: A tick slower than this is reported (one period of the G1's 50 Hz loop).
SLOW_MS = 20.0


def p99(times):
    return float(np.percentile(np.asarray(times) * 1e3, 99))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", help="dataset root (default: a temporary directory)")
    parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()
    root = Path(args.out or tempfile.mkdtemp(prefix="hirobot-record-"))

    robot = make_robot("unitree_g1_mujoco")
    robot.connect()
    try:
        robot.reset()
        emb = robot.embodiment
        hz = emb.control_hz
        pose = robot.get_observation().joint_pos.copy()
        joint = emb.joint_names.index("right_shoulder_pitch")
        steps = int(args.seconds * hz)
        wave(robot, hz, pose, joint)  # settle

        plain = []
        wave(robot, steps, pose, joint, plain)

        recorder = Recorder(robot, root, robot_config="unitree_g1_mujoco")
        events = []
        recorder.subscribe(lambda e: events.append(e) or print(
            f"  episode {e.episode}: {e.state}"
            + (f" {e.frames} frames, flags {[f.name for f in e.flags]}" if e.state == "saved"
               else "") + (f" ({e.error})" if e.error else "")))
        recording = []
        recorder.start("wave the right arm", meta={"operator": "example"})
        wave(robot, steps, pose, joint, recording)
        recorder.stop()
        after = []  # the loop goes on while the episode saves in the background
        while recorder.queue_length:
            wave(robot, 1, pose, joint, after)
        recorder.close()
    finally:
        robot.disconnect()

    print("control loop tick (get_observation + send_action):")
    for name, times in (("without recording", plain), ("recording", recording),
                        ("while saving", after)):
        ms = np.asarray(times) * 1e3
        print(f"  {name:17s} p99 {p99(times):6.2f} ms, max {ms.max():6.2f} ms, "
              f"{int((ms > SLOW_MS).sum())} of {len(ms)} ticks > {SLOW_MS:g} ms")
    saved = [e for e in events if e.state == "saved"]
    if not saved:
        raise SystemExit("the episode was not saved")
    check(Path(saved[0].path))


def check(path: Path) -> None:
    """Read the dataset back with pyarrow and PyAV and print what is in it."""
    import av
    import pyarrow.parquet as pq

    info = json.loads((path / "meta/info.json").read_text())
    hi_info = json.loads((path / "meta/hi/info.json").read_text())
    data = pq.read_table(path / "data/chunk-000/file-000.parquet")
    print(f"dataset {path}")
    print(f"  {info['codebase_version']}, {info['fps']} fps, {info['total_frames']} frames, "
          f"robot_type {info['robot_type']}, action mode {hi_info['hi.action_mode']}")
    state = info["features"]["observation.state"]["names"]
    print(f"  observation.state: {len(state)} dims ({state[0]} ... {state[-1]})")
    print(f"  columns: {', '.join(data.column_names)}")
    assert data.num_rows == info["total_frames"]
    for key, ft in info["features"].items():
        if ft["dtype"] != "video":
            continue
        video = path / info["video_path"].format(video_key=key, chunk_index=0, file_index=0)
        with av.open(str(video)) as f:
            n = sum(1 for _ in f.decode(video=0))
        print(f"  {key}: {n} frames {ft['shape'][1]}x{ft['shape'][0]} "
              f"{ft['info']['video.codec']}")
        assert n == info["total_frames"]
    report = json.loads((path / "meta/hi/alignment.json").read_text())
    for cam, r in report.get("cameras", {}).items():
        print(f"  {cam}: largest image offset {r['max_offset_s'] * 1e3:.1f} ms, "
              f"{r['violations']} frame(s) over {r['limit_s'] * 1e3:.1f} ms")
    print(f"  qc: {(path / 'meta/qc.jsonl').read_text().strip()}")


if __name__ == "__main__":
    main()
