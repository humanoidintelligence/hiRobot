# Universal Robots setup (UR5e / UR series)

English | [简体中文](./ur.zh-CN.md)

Driver: `ur_rtde` (`src/hirobot/drivers/body/ur/rtde/`). A 6-DoF fixed-base arm with no IMU
or base state. Wraps `ur_rtde` (`RTDEControlInterface` +
`RTDEReceiveInterface`), using `servoJ` to servo joints at high rate and
`servoL` to servo the TCP Cartesian pose.

> Why `ur_rtde` and not UR's official `RTDE_Python_Client_Library`: the latter
> is the low-level RTDE *protocol* library (register read/write). It needs a
> URScript program resident on the teach pendant that reads targets from
> registers and calls `servoj`. `ur_rtde` wraps exactly that pattern and
> exposes `servoJ/moveJ/speedJ/servoL` directly, which suits a driver better.

## Hardware status

Run on two real UR arms (2026-11-12), single arm and as the
`ur5e_dual` two-arm assembly: `get_observation` (joints, TCP pose) and
CARTESIAN `servoL` moving each TCP ±10 cm in Z, no protective stop, net drift
under 0.1 mm. The arms there are physically **UR5 (CB3)**, so they ran at
`control_hz: 125`; the driver and configs are the same for a UR5e, whose
e-Series controller runs up to 500 Hz. Not yet exercised on hardware: the
`reset()` / `moveJ`-home and `estop()` path, and POSITION / PD / VELOCITY servo
(those still only against a fake `ur_rtde`). Try each at low speed first.

## 1. Prerequisites

```bash
pip install ur_rtde
```

The driver can be constructed and run in sim without it; only a hardware
`connect()` needs it.

## 2. Network and robot setup

- Put the host on the same subnet as the UR control box. Note the control box
  IP and set it as `body.ip`.
- On the teach pendant, **enable External Control / RTDE** and make sure the
  robot is in Remote Control mode (otherwise the RTDE control interface cannot
  take over).
- e-Series arms such as the UR5e support up to 500 Hz; CB3 runs at 125 Hz. The
  config's `control_hz` must match (the driver uses `1/control_hz` as servoJ's
  time step `dt`).

## 3. Power-on and safety

1. Power on the robot, release the brakes, clear the workspace, and keep the
   e-stop within reach.
2. **Set a home pose first:** give `home_q` (6 joints, rad) in the config and
   `reset()` will `moveJ` there at `home_speed` / `home_acceleration` and wait
   until it arrives. Without it, `reset()` does not move. `estop()` from another
   thread stops the move (`stopJ` at `stop_deceleration`) and `reset()` raises
   `RobotStoppedError`. The `reset()` / `estop()` path has run only against a
   fake `ur_rtde`, not yet on hardware (see [Hardware status](#hardware-status)).
3. servoJ is a non-blocking servo; the control loop sends targets at the
   `control_hz` rate.
4. `disconnect()` calls `servoStop` + `stopScript` + `disconnect`; call it when
   you are done.

## 4. Configuration

`ur_ur5e`:

```yaml
embodiment:
  control_hz: 125           # e-Series can go up to 500
body:
  driver: ur_rtde
  ip: 192.168.1.102         # control box IP
  # home_q: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
  # servo_velocity: 0.5
  # servo_acceleration: 0.5
  # servo_lookahead: 0.1    # servoJ smoothing lookahead (s)
  # servo_gain: 300         # servoJ tracking gain
  # home_speed: 0.5         # moveJ to home_q (rad/s)
  # home_acceleration: 0.5  # (rad/s^2)
  # stop_deceleration: 2.0  # stopJ on estop() (rad/s^2)
  # command_timeout_s: 0.2  # speedStop when no new VELOCITY came for this long; null: off
```

Joint order: base → wrist (`shoulder_pan, shoulder_lift, elbow, wrist_1/2/3`),
matching RTDE `actual_q`.

**Two arms.** `ur5e_dual` assembles two `ur_ur5e` arms side by side as one
robot, the joints and TCP of each under a `left/` or `right/` prefix
(`left/shoulder_pan`, `right/tcp`). Both default to the `toy` driver; point each
at its own control box with per-part overrides:

```python
robot = make_robot("ur5e_dual", overrides={
    "left/body":  {"driver": {"type": "ur_rtde", "ip": "192.168.1.101"}},
    "right/body": {"driver": {"type": "ur_rtde", "ip": "192.168.1.102"}}})
```

One `Action` with `parts` commands both arms (each part reads its own
`tcp`); the assembly's `control_hz` is the outer loop, each arm's RTDE inner
loop is unchanged.

## 5. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config (built-in
config plus the driver and address above): read an observation, send the
current joint positions back as a POSITION target, and hold.

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_ur5e.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

What the driver does: `get_observation()` reads
`getActualQ/getActualQd/getActualCurrent` and `getActualTCPPose`. RTDE reports
the TCP as `[x,y,z,Rx,Ry,Rz]` (meters + axis-angle rad); the driver stores it in
`ee_poses["tcp"]` as the 7-vector `[x, y, z, qx, qy, qz, qw]`. POSITION / PD use
`servoJ`, VELOCITY uses `speedJ`, and CARTESIAN uses `servoL` (reading the
7-vector `ee_targets["tcp"]` and converting it back to axis-angle).

## 6. Common pitfalls

- **Remote Control mode:** if the robot is not in remote mode, the RTDE control
  interface cannot connect or take over.
- **Matching `dt`:** servoJ's `dt` comes from `control_hz`. The loop must keep
  up with that rate, or motion gets jerky or triggers a protective stop.
  e-Series arms commonly run at 500 Hz (dt = 2 ms).
- **Cartesian singularities / limits:** servoL to an unreachable or singular
  pose triggers a protective stop. Validate with small steps first.
- **Recovering from a stop:** after a protective stop or e-stop, clear it on
  the teach pendant and re-enable.
- **Only part of the driver is hardware-verified:** read and CARTESIAN `servoL`
  have run on a real arm; the `reset()` / `estop()` path and POSITION / PD /
  VELOCITY servo have not (fake `ur_rtde` only). See
  [Hardware status](#hardware-status) and try each mode at low speed first.
- **PD ignores `kp` / `kd`:** `PD` is accepted and tracked with `servoJ` like
  `POSITION`; servoJ's stiffness is the fixed `servo_gain`.
- **VELOCITY and `joint_limit`:** joint velocities go in `joint_target`, so a
  gate `joint_limit` bounds them in rad/s, not positions.
- **VELOCITY keeps moving:** `speedJ` runs until the next command. If the
  caller stops sending, the arm keeps its last speed; stop with `estop()` or a
  zero-velocity action.
- **RTDE libraries differ:** don't confuse `ur_rtde` (used by this driver) with
  UR's RTDE protocol library (`RTDE_Python_Client_Library`); the latter has a
  completely different API (registers + URScript).

## 7. RealSense camera (optional)

A UR arm has no camera of its own. An Intel RealSense plugged into the computer
running hiRobot is a `cameras` entry with the `intel_realsense` driver
(`src/hirobot/drivers/camera/intel/realsense/`), over `pyrealsense2`
(`pip install pyrealsense2`). It streams the colour sensor and, with
`depth: true`, the depth sensor aligned to colour, so `Observation.depth[name]`
shares the pixel grid of `Observation.images[name]` (metres, 0 where there is no
reading). Verified alongside the UR arms (2026-11-12) on a D435i and an L515,
both streaming at once. The D405 uses the same driver but has not been run.

Add it to your robot config, or attach it to the arm with the built-in
`intel/d435i_realsense` config:

```yaml
cameras:
  - name: camera
    from: camera/intel/d435i_realsense
    overrides:
      driver:
        serial: "<your-serial>"   # set when more than one RealSense is attached
```

The driver opens the first RealSense it finds; with more than one attached, set
`driver.serial` to the device serial (`pyrealsense2` enumerates it; `rs-enumerate-devices`
also prints it) so a fixed camera keeps its role. The camera is a read-only
source, so it also runs under `read_only`, and is skipped offline with the toy
driver.

### L515

The L515 has its own built-in config, `intel/l515_realsense`. Two things differ
from the D435i:

- **SDK version.** Newer `pyrealsense2` releases dropped the L515: 2.54.2 lists
  it, 2.59.1 does not (versions in between not tried). Install
  `pip install pyrealsense2==2.54.2.5684` (verified). 2.54.2 also runs a D435i, so one environment can drive both. With a
  newer `pyrealsense2` the L515 is simply missing from the device list, and
  opening it by serial fails.
- **Depth size.** On USB 2 the L515 offers depth only at 320x240 (colour up to
  640x480 at 30 fps), so the config sets `depth_width: 320`, `depth_height: 240`.
  Depth is still aligned to colour, so `Observation.depth` comes out at 640x480.
  Asking for depth at the colour size fails with `Couldn't resolve requests`.
  On USB 3 it offers larger depth sizes.

The first L515 depth frame arrives about 1.1 s after start, so the config sets
`timeout_ms: 2000`. The serial `pyrealsense2` reports for the L515 is a short
hex string (8 characters), not the 16-character USB serial `lsusb` shows.

```yaml
cameras:
  - name: scene
    from: camera/intel/l515_realsense
    overrides:
      driver:
        serial: "<your-serial>"
```

## 8. Gripper (Robotiq 2F)

A Robotiq 2F-85 (or 2F-140) on the tool flange is an `opening` end effector with
the `robotiq_urcap` driver (`src/hirobot/drivers/end_effector/robotiq/urcap/`).
It talks to the gripper through the Robotiq URCap installed on the UR
controller, which serves the gripper's registers on TCP port 63352 at the
controller's IP. No extra wiring or Python package is needed. The gripper
appears as `Observation.gripper["gripper"]` and is commanded with
`Action.gripper["gripper"]`, 0 closed and 1 open, in any control mode.

`ur_ur5e_robotiq` is a UR5e with the gripper on its `flange` mount. Point the arm
and the gripper at the same controller:

```python
robot = make_robot("ur_ur5e_robotiq", overrides={
    "body":    {"driver": {"type": "ur_rtde", "ip": "192.168.1.102"}},
    "gripper": {"driver": {"type": "robotiq_urcap", "host": "192.168.1.102"}}})
```

Driver options (`end_effector/robotiq/2f85_urcap`): `host`, `port` (63352),
`speed` and `force` (0-255, default 255), and `activate` (default `false`).

- **Activation.** By default the driver assumes the gripper is already
  activated (by the URCap on the pendant, or an earlier session); an
  unactivated gripper does not move. `activate: true` activates it on
  `connect()` if needed, which runs a full close/open stroke, so only use it
  with a clear workspace.
- **No motion feedback wait.** `command_opening` returns at once; read
  `get_state()` for the position. `extra["object"]` is Robotiq's object
  detection (0 moving, 1 or 2 stopped on an object, 3 at the target) and
  `extra["fault"]` its fault code.
- **No torque-off.** A Robotiq 2F has no passive mode, so `damp()` / `estop()`
  halt the motion and the gripper holds where it is.
- **Speed and force.** The defaults are the maximum. Start lower (for example
  `speed: 120`, `force: 80`) until you know the grip.

Hardware status: one close/open stroke through this driver on a 2F gripper on a
UR5 (CB3) controller (2026-11-12), position read back from fully open to the
closed stop and back, no fault. The UR5e config uses the same controller
interface.
