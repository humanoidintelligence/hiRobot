# Trossen WidowX 250S setup

English | [简体中文](./trossen_wx250s.zh-CN.md)

Driver: `trossen_dynamixel` (`src/hirobot/drivers/body/trossen/dynamixel/`). A
6-DoF arm with a parallel gripper, built from Dynamixel X-series servos on a
U2D2 USB-to-TTL adapter. The driver talks to the servos directly with Robotis'
[`dynamixel_sdk`](https://pypi.org/project/dynamixel-sdk/) (Protocol 2.0 at
1 Mbit/s). It needs no ROS and no Interbotix stack. The shipped config adds two
Intel RealSense D435i cameras (a wrist camera and a third-person scene camera,
as on a dual-arm teleop data-collection rig) through the `intel_realsense`
camera driver.

## Hardware status

Read on a real WidowX 250S follower arm (2026-11-15) with `dynamixel_sdk` 4.1.0:

- `connect()` and reading sent no torque and no goal (every servo's
  `Torque_Enable` stayed 0, the arm stayed limp). Joint positions, velocities,
  the gripper opening, motor temperatures, bus voltage and the per-servo
  torque/enable state all read back. Through the driver, the sleep pose read as
  shoulder −1.93, elbow 1.58, wrist_angle 0.92 rad, matching Interbotix's
  `wx250s.yaml` `sleep_positions`.

The gripper close/open ticks were calibrated on this arm (2026-11-15, torque
off, by hand): fully closed 1772, fully open 2957; the config uses 1780 / 2950
with a small margin off each hard stop. These were measured on one specific arm;
another may differ, so recalibrate per arm (see [section 3](#3-configuration)).

First commanded motion ran on this arm (2026-11-15, one joint at a time, a
person watching): the first `send_action` enabled torque with no jump and the
arm tracked POSITION targets, and it was folded back to its sleep pose
afterward. Two cautions from it. A joint raised far from its gravity-balanced
pose can stall and trip the servo's overload protection
(`Hardware_Error_Status` bit 5), which drops torque; clear it with a servo
reboot and reduce the speed. The servos move at whatever `Profile_Velocity` /
`Profile_Acceleration` is set — set a gentle profile before the first motion so
the arm eases into targets instead of lunging. `estop()` holding a moving arm
is still unexercised. Command motion carefully; see
[section 3](#3-configuration).

The gripper was exercised through the driver (`send_action` commanding opening
0.0 / 1.0 / 0.5 read back the same values) and one RealSense streamed colour and
aligned depth through the `intel_realsense` driver.

## 1. Prerequisites

```bash
pip install dynamixel_sdk       # 4.1.0 verified
```

The U2D2 shows up as a USB serial device. On an Interbotix station it is bound
to a stable name by udev rules: the follower arm is `/dev/ttyDXL`. Without those
rules it is `/dev/ttyUSB0`, `/dev/ttyUSB1`, ... in USB enumeration order. Check
which device answers the servos:

```bash
python -c "import dynamixel_sdk as d; p=d.PortHandler('/dev/ttyDXL'); \
p.openPort(); p.setBaudRate(1000000); \
print([i for i in range(1,10) if d.PacketHandler(2.0).ping(p,i)[1]==d.COMM_SUCCESS])"
# expect [1, 2, 3, 4, 5, 6, 7, 8, 9]
```

## 2. Power-on and safety

- Power the arm. With torque off (the state after power-on, and after this
  driver's `connect()`), the arm is limp and back-drivable; support it or let it
  rest in its cradle.
- Only one process may hold the serial port. On an Interbotix station the
  `xs_sdk` / teleop stack drives the same `/dev/ttyDXL`; stop it before running
  this driver, and check nothing else holds the port (`fuser /dev/ttyDXL`).
- Reading never moves the arm: `connect()`, `get_observation()` and
  `get_status()` enable no torque and send no goal.
- The **first** `send_action` enables torque. To avoid a jump, the driver first
  writes each servo's measured position as its goal, then enables torque, so the
  arm holds where it is; make the first commanded target near the current pose
  anyway.
- `estop()` writes the measured positions as the goal, so a moving arm stops
  where it is and stays powered. It does **not** enable torque: if the arm was
  limp, it stays limp. Keep a hand near the arm and the station's power switch.
- The shoulder (ID 2) and elbow (ID 4) each have a shadow servo (IDs 3, 5) that
  mirrors it in firmware (the Interbotix motor config sets each shadow's
  `Secondary_ID`). The driver commands and reads only the six primaries plus the
  gripper; the shadows follow on their own.

## 3. Configuration

`robots/trossen_wx250s` is the arm, its gripper and two RealSense D435i cameras
(wrist and scene), on the toy driver by default. For hardware, set the serial
port and each camera's serial in a local config:

```yaml
defaults: [robots/trossen_wx250s]
assembly:
  parts:
    body:  {overrides: {driver: {type: trossen_dynamixel, port: /dev/ttyDXL}}}
    wrist: {overrides: {driver: {serial: "<wrist camera serial>"}}}
    scene: {overrides: {driver: {serial: "<scene camera serial>"}}}
```

`body/trossen/wx250s` alone is the arm without cameras. Driver arguments:

| Argument | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyDXL` | Serial device the U2D2 is on |
| `servo_ids` | `(1, 2, 4, 6, 7, 8)` | Dynamixel IDs of the six arm joints, in embodiment order (shadows 3, 5 excluded) |
| `gripper` | `true` | Read and command the gripper; `false` leaves it alone |
| `gripper_id` | `9` | Dynamixel ID of the gripper servo |
| `gripper_close_tick` | `1780` | Servo position (ticks) at fully closed (`opening` 0) |
| `gripper_open_tick` | `2950` | Servo position (ticks) at fully open (`opening` 1) |
| `feedback_timeout_s` | `2.0` | How long `connect()` waits for every servo to ping |

**Gripper calibration.** The defaults (close 1780, open 2950) were measured on
one arm; another may differ. To recalibrate: with torque off, move
the gripper fully closed and fully open by hand and read `Present_Position`
(address 132) on the gripper servo at each end, then put the two ticks in the
config. The opening is a linear map between them, so the sign takes care of
itself whichever way the servo turns.

Joint names (`waist`, `shoulder`, `elbow`, `forearm_roll`, `wrist_angle`,
`wrist_rotate`) and limits come from Interbotix's `wx250s` motor config and URDF.
Positions convert as `rad = (ticks − 2048) · 2π / 4096`, one formula for every
servo: Interbotix sets each servo's `Drive_Mode` reverse bit in firmware so every
joint already counts in its URDF-positive direction.

## 4. Observation and action

| Field | Source | Units |
|---|---|---|
| `joint_pos` | `Present_Position` (addr 132) of each primary servo | rad |
| `joint_vel` | `Present_Velocity` (addr 128) | rad/s |
| `gripper["ee"]` | gripper servo position mapped between close/open ticks | 0 closed, 1 open |
| `get_status().joint_temperature` | `Present_Temperature` (addr 146) | °C |
| `get_status().extra` | torque, voltage, hardware-error registers | `enabled`, `voltage_v`, `hardware_error` |

Current, velocity and position come from one `GroupSyncRead` over the contiguous
block (addresses 126-135). `joint_torque` and `ee_poses` are not filled in (the
driver has no kinematics; a `Kinematics` layer can add the flange pose from
`joint_pos`).

Actions: `ControlMode.POSITION` with `joint_target` (6 values, rad) and/or
`gripper={"ee": opening}`. Each command enables torque on the first call, then
writes `Goal_Position` (addr 116) to every servo.

## 5. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config: read an
observation, send the current joint positions back as a POSITION target, and
hold.

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_wx250s.yaml")
robot.connect()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos,
                         gripper={"ee": obs.gripper["ee"]}))
robot.disconnect()
```

The first `send_action` enables torque. Run it only when the arm is supported and
free to hold its pose.

## 6. RealSense cameras

Driver: `intel_realsense` (`src/hirobot/drivers/camera/intel/realsense/`), config
`camera/intel/d435i_realsense`. It streams colour and, with `depth: true`, depth
aligned to colour, in metres. With two RealSense attached, set `wrist.driver.serial`
and `scene.driver.serial` so each camera keeps its role (`rs-enumerate-devices`
lists the serials). See [ur.md](ur.md) for the shared RealSense notes.

## Troubleshooting

- **`could not open /dev/ttyDXL`**: the device name is wrong, or another process
  (the Interbotix `xs_sdk`) holds the port. Stop that stack and check `fuser`.
- **`no answer from servo(s) [...]`**: the arm is unpowered, the baud rate is not
  1 Mbit/s, or a listed `servo_id` is wrong. Ping the IDs as in
  [section 1](#1-prerequisites).
- **Gripper reads stuck at 0 or 1**: the close/open ticks are not calibrated for
  this arm; measure them ([section 3](#3-configuration)).
