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
   `RobotStoppedError`. This path has run only against a fake `ur_rtde`.
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
- **Not run on a real arm yet:** the driver is tested against a fake `ur_rtde`
  only. Try each mode at low speed first.
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
