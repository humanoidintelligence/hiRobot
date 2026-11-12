# AgileX PiPER setup

English | [简体中文](./agilex_piper.zh-CN.md)

Driver: `agilex_piper_sdk` (`src/hirobot/drivers/body/agilex/piper_sdk/`). A 6-DoF
arm with a parallel gripper on the same CAN bus. It wraps AgileX's
[`piper_sdk`](https://github.com/agilexrobotics/piper_sdk) (`C_PiperInterface_V2`)
over SocketCAN. The scene camera in the shipped config is an Orbbec Dabai DC1
through the `orbbec_pyorbbecsdk` camera driver ([section 6](#6-orbbec-camera)).

## Hardware status

Run on a real PiPER (2026-11-12), the right arm of a two-arm station, with
`piper_sdk` 0.6.1:

- `connect()` and reading sent nothing (the CAN interface's transmit counter
  did not change). Joints, the flange pose, the gripper stroke, motor
  temperatures, voltage and the enable state all read back.
- A POSITION move raised the flange 99 mm (joint2 +11°, joint3 −19°, 20 %
  speed) and returned it. The gripper closed to 20 mm and reopened. The arm
  ended within 0.2° of where it started.
- Through `make_robot`, the arm, gripper and a Dabai DC1 (colour 640x480 with
  aligned depth) were read together.

Not yet run on hardware: `estop()`, and one command sent while a previous move
is still in progress (`JointCtrl` replaces the arm's target as it goes).

## 1. Prerequisites

```bash
pip install piper_sdk            # 0.6.1 verified
sudo apt install can-utils       # candump, to check the bus
```

The PiPER's USB-to-CAN adapter shows up as a SocketCAN interface (`can0`,
`can1`, ... on a station with several arms). Bring it up at 1 Mbit/s after
every boot or replug:

```bash
sudo ip link set can0 up type can bitrate 1000000
candump can0 -n 5      # frames 0x2A1..0x2A8 (state) and 0x251..0x256 (motors)
```

With several adapters, the `can<N>` names follow USB enumeration order.
AgileX's `piper_sdk` ships `can_activate.sh`, which names an interface by USB
port so each arm keeps its name.

## 2. Power-on and safety

- Power the arm and let its controller boot; `candump` shows feedback at
  ~200 Hz once it is up.
- The driver sends nothing until the first command, so `connect()` and
  `get_observation()` can run on an arm someone else is driving. To keep it
  that way, use `read_only: true` ([api.md](../api.md)).
- The first command enables any motors that are off (`EnableArm`). Check that
  the target is near the current pose before the first command.
- Moves are the controller's own joint moves (`MOVE J`) at `speed_percent` of
  its top speed (default 30 %). The controller plans the trajectory, so a far
  target is reached at that speed rather than in one step.
- `estop()` sends a joint move to the measured position, so the arm stops where
  it is and stays powered. It does not use the controller's quick stop
  (`EmergencyStop`), which lets the arm sink under damping. Keep the
  station's hardware stop within reach.

## 3. Configuration

`robots/agilex_piper` is the arm, its gripper and a scene camera, on the toy
driver by default. For hardware, put the CAN interface and the camera in a
local config:

```yaml
defaults: [robots/agilex_piper]
assembly:
  parts:
    body: {overrides: {driver: {type: agilex_piper_sdk, can: can0}}}
    scene: {overrides: {driver: {serial: CC1N00000000}}}
```

`body/agilex/piper` alone is the arm without a camera. Driver arguments:

| Argument | Default | Meaning |
|---|---|---|
| `can` | `can0` | SocketCAN interface the arm is on |
| `speed_percent` | `30` | Speed of joint moves, 1-100 % of the arm's top speed |
| `gripper` | `true` | Read and command the gripper; `false` leaves it alone |
| `gripper_max_m` | `0.07` | Stroke that `Action.gripper = 1` opens to |
| `gripper_effort_n` | `1.0` | Gripping force, 0-5 N |
| `feedback_timeout_s` | `2.0` | How long `connect()` waits for joint feedback |

Joint names (`joint1` .. `joint6`) and limits come from AgileX's
`piper_description` URDF and `piper_sdk`'s joint limits.

## 4. Observation and action

| Field | Source | Units |
|---|---|---|
| `joint_pos` | joint feedback (0x2A5-0x2A7) | rad |
| `ee_poses["ee"]` | the flange pose the controller computes (0x2A2-0x2A4) | `[x, y, z, qx, qy, qz, qw]`, m, in the arm's base frame |
| `gripper["ee"]` | gripper stroke (0x2A8) / `gripper_max_m` | 0 closed, 1 open |
| `get_status().joint_temperature` | motor feedback (0x261-0x266) | °C |
| `get_status().extra` | arm status (0x2A1) and motor feedback | `ctrl_mode`, `arm_status`, `motion_status`, `enabled`, `voltage_v` |

The controller reports the flange orientation as Euler angles `RX, RY, RZ`
(0.001°) with `R = Rz(RZ) Ry(RY) Rx(RX)`; the driver converts them to a
quaternion. `joint_vel` and `joint_effort` are not filled in.

Actions: `ControlMode.POSITION` with `joint_target` (6 values, rad) and/or
`gripper={"ee": opening}`. Each command sends `MotionCtrl_2` (CAN control,
`MOVE J`, `speed_percent`) then `JointCtrl`; a gripper value sends
`GripperCtrl`.

## 5. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config: read an
observation, send the current joint positions back as a POSITION target, and
hold.

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_piper.yaml")
robot.connect()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos,
                         gripper={"ee": obs.gripper["ee"]}))
robot.disconnect()
```

## 6. Orbbec camera

Driver: `orbbec_pyorbbecsdk` (`src/hirobot/drivers/camera/orbbec/pyorbbecsdk/`),
config `camera/orbbec/dabai_dc1`. It streams colour and, with `depth: true`,
depth aligned to colour, in metres.

Older Orbbec cameras such as the Dabai DC1 only enumerate with the **1.x**
Python bindings over OrbbecSDK 1.10; the 2.x bindings (`pyorbbecsdk2`) do not
list them. Build the 1.x bindings from source (the `v1` line of
[pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk)) and put the build's
`install/lib` on `PYTHONPATH`. Install Orbbec's udev rules so the camera opens
without root.

- **numpy.** Run the bindings under the numpy major version they were built
  against. A 1.x build running under numpy 2 hands out colour buffers that are
  all `0xff`; the driver then reports that it could not decode MJPEG and names
  the cause. `opencv-python` 4.12 and later require numpy 2, so with a 1.x
  build use `numpy<2` and `opencv-python<4.12`.
- **Choosing a camera.** Set `serial` (printed on the camera) or `uid` (the
  USB port path the SDK lists, e.g. `3-1.1.3.4-16`). The Dabai DC1 leaves its
  serial empty in the device list, so with `serial` the driver opens each
  unnamed device to read it. That can interrupt a camera another process is
  streaming; use `uid` on a shared machine.
- **Alignment.** `align: hw` (default) aligns depth to colour on the camera.
  The Dabai DC1 does this only at 640x480 colour; at 1280x720 use `align: sw`.
  The driver raises an error when depth does not match the colour size.
- **One process per camera.** A camera that another program is streaming
  cannot be opened; stop that program first.

Driver arguments: `serial`, `uid`, `width` / `height` (640x480), `fps` (30),
`color_format` (`MJPG`, `RGB` or `YUYV`), `depth`, `align`, `timeout_ms`,
`poll_hz`.

## 7. Common pitfalls

- **`no PiPER joint feedback on can0`**: the interface is down, at the wrong
  bitrate, or the arm is off. Check `ip -details link show can0` (state UP,
  bitrate 1000000) and `candump can0`.
- **Wrong arm moved**: interface names follow USB order. Check which arm is on
  which interface by reading it before sending anything.
- **`motion_status` says `REACH_TARGET_POS_FAILED`** right after a move: the
  controller sets this while a joint is still settling; it reads
  `REACH_TARGET_POS_SUCCESSFULLY` once the arm settles.
