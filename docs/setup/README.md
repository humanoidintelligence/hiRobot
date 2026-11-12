# Robot setup

English | [简体中文](./README.zh-CN.md)

Steps for each supported robot, from power-on to sending the first command
through hiRobot. Each guide covers prerequisites (SDK, network), power-on and
safety, connection parameters, the minimal hiRobot loop, and common pitfalls.

> Before touching hardware, get the control loop running under `driver: toy`
> (see the Quick start in the main README) to confirm the embodiment and policy
> are wired correctly. Then switch `body.driver` in the config to the hardware
> driver.

## Supported robots

| Robot | Type | Driver | SDK | Config | Guide |
|---|---|---|---|---|---|
| Unitree G1 | humanoid, 29 DoF | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_g1` | [unitree.md](unitree.md) |
| Unitree H1-2 | humanoid, 27 DoF | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_h1_2` | [unitree.md](unitree.md) |
| Unitree H2 | humanoid, 31 DoF | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_h2` | [unitree.md](unitree.md) |
| Unitree H1 | humanoid, 19 DoF | `unitree_sdk2` | unitree_sdk2py (go) | `unitree_h1` | [unitree.md](unitree.md) |
| Fourier GR1T1 | humanoid, 32 DoF | `fourier_grx` | fourier-grx-client | `fourier_gr1t1` | [fourier.md](fourier.md) |
| Fourier GR1T2 | humanoid, 32 DoF | `fourier_grx` | fourier-grx-client | `fourier_gr1t2` | [fourier.md](fourier.md) |
| AgiBot G1 | dual arm on a wheeled base, 18 DoF | `agibot_gdk` | GDK topics (cosine_bus) | `agibot_g1` | [agibot_g1.md](agibot_g1.md) |
| AgiBot X1 | humanoid, 29 DoF | `agibot_aimrt` | ROS2 + AimRT bridge | `agibot_x1` | [agibot_x1.md](agibot_x1.md) |
| Franka FR3 | arm with gripper, 7 DoF, on a DROID station | `franka_droid` | zerorpc to a DROID robot server | `franka_fr3_droid` | [franka.md](franka.md) |
| UR5e | arm, 6 DoF | `ur_rtde` | ur_rtde | `ur_ur5e` | [ur.md](ur.md) |
| AgileX PiPER | arm with gripper, 6 DoF | `agilex_piper_sdk` | piper_sdk | `agilex_piper` | [agilex_piper.md](agilex_piper.md) |

## General workflow

1. **Install the SDK.** Every driver imports its vendor SDK **lazily**: the
   driver can be constructed and run in sim without it, and the SDK is only
   needed when `connect()` talks to hardware. Install the SDK listed in each
   guide. `connect()` first checks it against the driver's `manifest.yaml`
   (`src/hirobot/drivers/<part>/<vendor>/<sdk>/`): a missing SDK, or one older than
   the manifest's `minimum`, stops with the install command; one newer than any
   version listed under `verified` logs a warning and runs.
2. **Network and power-on.** See "Power-on and safety" in each guide.
3. **Set connection parameters.** Write a local config that starts with
   `defaults: [<robot>]` (the built-in config name, see `hirobot configs`) and
   set the driver name and connection parameters (network interface / IP /
   `server_ip`) under `body:`. These keys are passed to the driver constructor
   as-is. `hirobot info --config my_robot.yaml --resolved` shows the merged result.
4. **Run the minimal loop.** Open the robot with your config, read an
   observation and send the current joint positions back as a POSITION target
   (the loop is in the [Quick start](../../README.md#simulation)). Each guide
   shows it for its robot.

> ⚠️ **Safety:** on first power-on a humanoid must be hung or supported. Start
> in damping mode, and always make the control stack's first step a ramp from
> the **measured** pose, never a jump to a target. Each driver's `reset()`
> already ramps, but someone must still stand by the e-stop.
