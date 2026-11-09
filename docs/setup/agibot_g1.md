# AgiBot G1 setup (GDK topics, chassis controller)

English | [简体中文](./agibot_g1.zh-CN.md)

Drivers: `agibot_gdk` body (`src/hirobot/drivers/body/agibot/gdk/`) and
`agibot_cosine` cameras (`src/hirobot/drivers/camera/agibot/cosine/`).
Config: `agibot_g1`.

AgiBot G1 (精灵 G01) is a dual-arm robot on a wheeled chassis: two 7-DoF arms
with Omnipicker grippers, a waist (lift and pitch), a head (yaw and pitch), a
head RGB-D camera and a colour camera on each wrist. hiRobot runs **on the
robot's own computer**, next to the vendor's motion controller, and talks to it
over the GDK's DDS topics. The chassis has its own controller box on the robot
network; the driver reaches it over HTTP and WebSocket.

The driver does not use the vendor's `a2d_sdk.robot` Python layer: that layer
needs ruckig and scipy and a newer message package than the robot's system
Python ships. The driver subscribes and publishes on the same topics directly.

## 1. Prerequisites

On the robot's computer (Ubuntu 22.04, system Python 3.10, `/usr/bin/python3`):

| Package | Where it comes from |
|---|---|
| `cosine_bus` 2.0.0 | preinstalled |
| `websockets`, `numpy`, `pydantic`, `PyYAML`, `opencv-python` | preinstalled on the tested robot |
| `genie_msgs_pb` >= 0.8.0 | in the GDK installer the robot carries; see below |

The system message package may be older than 0.8.0. The GDK installer on the
robot (`a2d_sdk/gdk_installer/gdk_device_installer.tar.gz` under the vendor's
app directory) holds `genie_msgs_pb-0.8.0-py3-none-any.whl`. Extract the wheel
to a directory of your own and put it on `PYTHONPATH`; it does not need to be
installed into the system Python:

```bash
mkdir -p <dir>/vendor && cd <dir>/vendor
tar xzf <path to>/gdk_device_installer.tar.gz gdk_device_installer/genie_msgs_pb-0.8.0-py3-none-any.whl
python3 -m zipfile -e gdk_device_installer/genie_msgs_pb-0.8.0-py3-none-any.whl genie_msgs_pb-0.8.0
```

Install hiRobot without dependencies, so the system packages stay as they are:

```bash
/usr/bin/python3 -m pip install --user --no-deps <path to hiRobot>
```

Running from a source copy with `PYTHONPATH=<hiRobot>/src` works too.

Every shell that runs hiRobot needs the vendor environment, which sets the
library path and the DDS discovery variables (`LOCATOR_IP`,
`AORTA_DISCOVERY_URI`):

```bash
source <vendor app dir>/env.sh <vendor app dir>/lib
export PYTHONPATH=<dir>/vendor/genie_msgs_pb-0.8.0:$PYTHONPATH
```

Without it, `connect()` times out waiting for the state topics, and the camera
driver names the missing variables.

## 2. Power-on and safety

- Keep the area around the arms and about 0.5 m around the chassis clear, and
  keep the e-stop in reach.
- **The motion controller has three modes: STOP, SERVO and PLANNING.**
  `connect()` only reads. `reset()` asks for SERVO when the controller is in
  STOP; it moves no joint. `send_action()` refuses unless the controller is in
  SERVO, an e-stop is pressed, it reports an error, or the arm state is older
  than `max_state_age`. `damp()` and `disconnect()` send `MotionStop`, which
  stops the joints where they are and puts the controller in STOP.
- **Only the joint groups in `groups` are commanded** (default: both arms).
  A target for a waist or head joint must equal its measured position (within
  `hold_tolerance`), or the action is refused; nothing is dropped silently.
- Commands carry a `lifetime` (0.2 s): when they stop coming, the motion
  controller stops following them.
- The chassis box only accepts speed in **soft-manual** mode. Switch it on the
  chassis controller's web page; the driver never changes it. In soft-manual
  the box reports `isHardAuto=true` and `isSoftAuto=false`; with any other
  combination the driver refuses a non-zero `BASE_VELOCITY`.
- The driver resends the latest base speed every 100 ms and switches to zero
  speed when no new `BASE_VELOCITY` came for `command_timeout` (0.3 s).
- Use `read_only=True` for a first look: it creates no publisher, and commands
  and `reset()` raise.

## 3. Configuration

`agibot_g1` (with `body/agibot/g1` and `camera/agibot/g1`):

```yaml
body:
  driver: toy              # set to 'agibot_gdk' on the robot's computer
  # groups: [left_arm, right_arm]   # joint groups agibot_gdk commands
  # chassis_host: <chassis box IP>  # enables base state and BASE_VELOCITY
safety:
  gripper_range: [0, 1]
```

Driver arguments (`AgibotG1Robot`):

| Argument | Default | Meaning |
|---|---|---|
| `groups` | `[left_arm, right_arm]` | Joint groups to command: `left_arm`, `right_arm`, `waist`, `head` |
| `read_only` | `false` | No publisher; commands and `reset()` raise |
| `lifetime` | `0.2` | How long the controller follows one command (s) |
| `hold_tolerance` | `0.02` | How far a target for an uncommanded joint may be from its measured position (rad or m) |
| `max_state_age` | `0.2` | Refuse to command on older arm state or motion status (s) |
| `connect_timeout` | `3.0` | Seconds to wait for every state topic |
| `gripper_raw_closed` | `120.0` | Raw gripper reading when fully closed |
| `chassis_host` | none | Chassis controller box address; without it there is no chassis |
| `chassis_id` | `780` | The `id` the chassis box answers to (on its web page) |
| `command_timeout` | `0.3` | Seconds after the last `BASE_VELOCITY` before zero speed |
| `max_linear_speed` | `0.2` | Largest accepted `vx` (m/s) |
| `max_angular_speed` | `0.5` | Largest accepted `wz` (rad/s) |

Joints, in config order: `waist_lift_joint` (prismatic, metres),
`waist_pitch_joint`, `head_yaw_joint`, `head_pitch_joint`,
`left_arm_joint1` ... `left_arm_joint7`, `right_arm_joint1` ...
`right_arm_joint7`. They are the motion controller's `a2d_v01` model names
without the `idxNN_` prefix. `position_limit` comes from the running
controller's own configuration and is tighter than the URDF for the elbows
(`joint4`) and the waist lift.

Cameras (`agibot_cosine`):

| Name | Camera | Topic | Image | Depth |
|---|---|---|---|---|
| `head` | RealSense D457 | `/camera/head_color`, `/camera/head_depth` | 1280×800 RGB | metres, paired with the colour frame of the same capture |
| `left_wrist` | RealSense D405 | `/camera/hand_left_color` | 848×480 RGB | none: the vendor camera service has wrist depth turned off (`depthEnable: false`) |
| `right_wrist` | RealSense D405 | `/camera/hand_right_color` | 848×480 RGB | none |

## 4. Observation and action

| Field | Content |
|---|---|
| `joint_pos` | 18 joints from the arm, waist and neck state topics |
| `joint_vel` | `None` on the tested firmware (the arm topic's velocity count does not match its joints) |
| `gripper` | `left_gripper`, `right_gripper`: 0 closed, 1 open |
| `ee_poses` | `left_gripper`, `right_gripper`: `arm_{left,right}_link7` in `base_link`, `[x, y, z, qx, qy, qz, qw]`, as the motion controller reports it |
| `base_position`, `base_orientation` | chassis odometer (with `chassis_host`) |
| `images`, `depth` | the cameras above |

`get_status()` gives the chassis battery and, in `extra`, the motion mode, the
per-part error codes and e-stops, and the chassis flags (`isSoftAuto`,
`isHardAuto`, `isScram`, `isPause`, `exceptions`).

Actions:

- `POSITION`: `joint_target` for all 18 joints, plus `gripper` (0 closed,
  1 open). The driver converts the gripper to the GDK's scale (0 open, 1 closed).
- `BASE_VELOCITY`: `[vx, 0, wz]` in m/s and rad/s; `vy` must be 0. A positive
  `wz` turns counter-clockwise seen from above.

## 5. Minimal loop

Read only first:

```python
from hirobot import make_robot

robot = make_robot("agibot_g1", body_overrides={"driver": "agibot_gdk", "read_only": True,
                                                "chassis_host": "<chassis box IP>"})
robot.connect()
obs = robot.get_observation()
print(obs.joint_pos, obs.gripper, obs.base_position, {k: v.shape for k, v in obs.images.items()})
print(robot.get_status())
robot.disconnect()
```

Then a small move, ramped from the measured pose (here the right shoulder,
`right_arm_joint1`, +10° over 3 s at 30 Hz; positive raises the arm forward):

```python
import math, time
import numpy as np
from hirobot import Action, ControlMode, make_robot

robot = make_robot("agibot_g1", body_overrides={"driver": "agibot_gdk"})
robot.connect()
robot.reset()                              # STOP -> SERVO; moves no joint
q0 = robot.get_observation().joint_pos.copy()
j = robot.embodiment.joint_names.index("right_arm_joint1")
for k in range(91):
    q = q0.copy()
    q[j] += math.radians(10) * 0.5 * (1 - math.cos(math.pi * k / 90))
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    time.sleep(1 / 30)
robot.disconnect()                         # MotionStop: the arm stays where it is
```

## 6. Common pitfalls

- **`connect` times out on the state topics:** the vendor environment is not
  sourced in this shell, or the motion controller is not running.
- **`ImportError` for `genie_msgs_pb`:** the 0.8.0 package is not on
  `PYTHONPATH` (section 1).
- **`send_action` says the controller is not in SERVO:** call `reset()`. If
  the mode stays in STOP, another controller (the HMI, teleoperation) holds the
  robot, or the controller reports an error; `get_status().extra` shows the
  error codes.
- **Base motion refused with "not in soft-manual mode":** switch the chassis to
  soft-manual on its web page.
- **The chassis is slow to start and keeps moving after a stop:** a turn at
  0.2 rad/s barely moved for the first 0.8 s and turned another ~7° after zero
  speed was sent. To stop at a heading or a distance, stop early and read the
  odometer after it settles.
- **The head neck state reports an error code** (`hal.neck_error` in
  `get_status().extra`) on the tested robot. The head is not commanded by
  default; check the head on the vendor HMI before adding `head` to `groups`.
- **Wrist depth:** the wrist depth topics exist but publish no frames: the
  vendor camera service's configuration turns wrist depth off
  (`depthEnable: false`). hiRobot has not tried turning it on.

## 7. Tested

On a G1 with firmware `a2d_G01_2.3.4`, `cosine_bus` 2.0.0, `genie_msgs_pb`
0.8.0, chassis firmware 2.0.0.7:

| Test | Result |
|---|---|
| Read only | 18 joints match the raw topics; grippers, wrist poses, odometer, battery |
| Cameras | head 1280×800 RGB + depth at 21–25 frames/s; wrists 848×480 at 28–30 frames/s |
| Grippers | both opened (read 0.998 / 1.0) and closed (0 / 0) over 2 s each |
| Right shoulder | `right_arm_joint1` +10° and +30° and back; tracked within 0.02°, other joints < 0.001 rad |
| Chassis | 0.05 m/s forward 103 mm and back 101 mm by odometer; a clockwise turn at 0.2 rad/s turned 25° |
