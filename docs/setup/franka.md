# Franka setup (FR3 / Panda)

English | [简体中文](./franka.zh-CN.md)

Driver: `franka_droid`. hiRobot drives a Franka FR3 through a DROID robot server;
it does not talk to the arm over libfranka itself.

On a [DROID](https://github.com/droid-dataset/droid) station the arm and its
gripper are already run by DROID's robot server (`scripts/server/run_server.py`
on the control computer, zerorpc on port 4242). The `franka_droid` driver
(`src/hirobot/drivers/body/franka/droid/`) is a client of that server, so hiRobot
needs no RT kernel, no libfranka and no direct FCI connection; it only needs
`pip install zerorpc`. It calls only methods upstream DROID has
(`get_robot_state`, `update_joints`, `update_pose`, `update_gripper`), and does
not start or stop the server's controller. Start the server the DROID way.

## Configuration

Configs: `body/franka/fr3_droid` (arm plus gripper on the `ee` frame,
`control_hz: 15` as in DROID) and `robots/franka_fr3_droid` (that body, a ZED
Mini `wrist` camera and two ZED 2i, `left` and `right`). Put the server address
and camera serial numbers in a local config:

```yaml
defaults: [robots/franka_fr3_droid]

assembly:
  parts:
    body: {overrides: {driver: {type: franka_droid, ip: 172.16.0.2}}}
    wrist: {overrides: {driver: {serial: <wrist ZED serial>}}}
    left: {overrides: {driver: {serial: <left ZED serial>}}}
    right: {overrides: {driver: {serial: <right ZED serial>}}}
```

The ZED cameras use the `stereolabs_zed` camera driver, on the computer they are
plugged into (it needs the ZED SDK, its `pyzed` wheel and an NVIDIA GPU). A ZED
can be open in only one process, so stop DROID's own camera reader first.

## What the driver does

| | `franka_droid` |
|---|---|
| Arguments | `ip` (default `172.16.0.2`), `port` (4242), `timeout_s` (10), `home_q` (7 joint angles for `reset()`, optional), `gripper` (`true`) |
| `get_observation()` | `joint_pos`, `joint_vel`, `ee_poses["ee"]` (DROID's `cartesian_position`, Euler `xyz` angles converted to a quaternion), `gripper["ee"]` (DROID's 0 open / 1 closed turned into 0 closed / 1 open) |
| `POSITION` | `update_joints(q, velocity=False, blocking=False)`: the server's controller tracks the joint target |
| `CARTESIAN` | `ee_targets["ee"]` → `update_pose(pose, velocity=False, blocking=False)`; the server does the IK or impedance control |
| `Action.gripper["ee"]` | `update_gripper(1 - opening, velocity=False, blocking=False)`, in either mode |
| `reset()` | With `home_q`, a blocking `update_joints` to it; otherwise nothing |
| `estop()` | Sends the measured joints as the target, so the arm holds where it is. The server's controller keeps running; use the Franka's own stop button for a hard stop |

Each OS thread gets its own zerorpc client (zerorpc runs on gevent), so
`estop()` from another thread does not wait on the control loop's call.
`connect()` reads the state once and raises `ConnectionError` if the server
does not answer.

## Verified on hardware

Verified on a DROID station (FR3, Robotiq 2F-85, three ZEDs, 2026-11-12):
reads at 15 Hz with a median call time of 2.4 ms; a `CARTESIAN` move 10 cm up
and back at up to 4 cm/s, streamed at 15 Hz, tracked within 6.3 mm and ended
1.1 mm from the start; the gripper went to 0.50 (measured 0.502) and back;
`make_robot` read the arm and three ZEDs (1280x720 colour plus depth) together.

## Common pitfalls

After the connection between the server and the arm drops (the server
reports `Socket is not connected`), every state read fails until the server
recovers the arm. That is the server's job; `franka_droid` does not try.
