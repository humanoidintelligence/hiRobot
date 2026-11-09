# Franka setup (FR3 / Panda)

English | [简体中文](./franka.zh-CN.md)

Driver: `franka_libfranka` (`src/hirobot/drivers/body/franka/libfranka/`). A 7-DoF fixed-base arm with
no IMU or base state. Wraps `pylibfranka`, the official pybind bindings of
[libfranka](https://github.com/frankarobotics/libfranka).

libfranka's streaming controllers require a **1 kHz real-time loop on a
PREEMPT_RT kernel**. To keep hiRobot's soft-real-time loop usable, the driver
uses the **asynchronous joint-position controller**
(`AsyncPositionControlHandler`): each `send_action` sets a new joint target that
libfranka's own RT thread tracks, so a VLA policy can issue targets at its own,
lower rate.

## 1. Prerequisites

Install the bindings (pre-built wheels; see `pylibfranka/README.md` in the
libfranka repo):

```bash
pip install pylibfranka
# unsupported platform: build from source per libfranka's pylibfranka/README.md (CMake + pybind11)
```

You need a libfranka version that matches the robot firmware, and a PREEMPT_RT
kernel (required for production control).

## 2. Network and robot setup

- Connect the host to the robot's FCI port on the FCI subnet. Note the FCI
  address (often `172.16.0.2`) and set it as `body.ip`.
- In Desk (Franka's web UI): unlock the joints, activate **FCI**, and make sure
  the robot is **not held by Desk**.
- The status light must show the robot is ready for external control (blue).

## 3. Power-on and safety

1. Clear the workspace and keep the e-stop (and Franka's enabling device) within reach.
2. `connect()` sets conservative collision thresholds
   (`set_collision_behavior`); tune them for your payload.
3. `reset()` calls `automatic_error_recovery()` to clear reflexes / faults and
   **starts the async position controller**. So **on hardware, call `reset()`
   before `send_action()`**, or the driver raises an error telling you to reset.
4. `realtime=False` relaxes `RealtimeConfig` when there is no RT kernel. Use it
   only for setup checks, **never for production control**.

## 4. Configuration

`franka_fr3`:

```yaml
embodiment:
  control_hz: 100           # rate the VLA sends targets; libfranka's RT thread tracks at 1 kHz
body:
  driver: franka_libfranka
  ip: 172.16.0.2            # FCI address
  # max_joint_velocity: 1.0 # per-joint velocity cap of the async controller (rad/s)
  # goal_tolerance: 0.01    # goal tolerance (rad)
  # realtime: true          # set false without an RT kernel (setup only)
```

Joint order: `joint1..joint7`, matching libfranka's `RobotState.q`.

## 5. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config (built-in
config plus the driver and address above): read an observation, send the
current joint positions back as a POSITION target, and hold.

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_franka.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

What the driver does: `get_observation()` reads `q/dq/tau_J` from
`read_once()` and converts the 4x4 column-major `O_T_EE` (base frame) into the
7-vector pose `[x, y, z, qx, qy, qz, qw]` in `ee_poses["ee"]`.
`send_action()` (POSITION / PD) calls `handler.set_joint_position_target(...)`.

## 6. Common pitfalls

- **Reset first:** the async controller starts in `reset()`; calling
  `send_action` without it raises an error.
- **RT kernel:** with `realtime=True` (the default) and no RT kernel, libfranka
  raises. During setup you can temporarily set `realtime=False`.
- **POSITION / PD only:** this driver uses async joint-position control.
  Cartesian and torque control are not wrapped (libfranka supports them; extend
  with `start_cartesian_*` / torque controllers).
- **Collision thresholds:** the defaults are conservative; heavy payloads or
  fast motions easily trigger a reflex stop. Adjust `set_collision_behavior`
  in the driver as needed.
- **Desk holds the robot:** if a Desk program is running or FCI is not
  activated, external control cannot connect.
- **Not run on a real arm yet:** the driver is tested against a fake
  `pylibfranka` only. Not verified on hardware: that `pylibfranka` offers
  `start_async_position_control` / `set_joint_position_target` as the driver
  calls them, that `read_once()` works while the async controller runs, and what
  the arm does after `estop()` stops the controller. libfranka also raises the
  constructing thread (the Python main thread) to real-time priority, and
  threads started from it inherit that.
- **`reset()` does not move:** it clears errors and restarts the controller;
  there is no home pose.
