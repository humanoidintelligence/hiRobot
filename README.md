# hiRobot: A Unified Interface Layer for Controlling Diverse Robots from VLA Models and Agents

<div align="center">

<img src="assets/teaser.png" alt="Supported robot vendors: Unitree, Fourier, AgiBot, Franka, Universal Robots">

**Author:** [The Humanoid Intelligence Team](https://humanoidintelligence.github.io) from [OpenMOSS Group](https://openmoss.ai) / [Shanghai Innovation Institute](https://www.sii.edu.cn/) / [Fudan University](https://www.fudan.edu.cn/en/)

English | [简体中文](./README.zh-CN.md)

</div>

<!-- toc -->

<details>
<summary><b>Contents</b></summary>

- [Introduction](#introduction)
- [Architecture](#architecture)
- [Repository structure](#repository-structure)
- [Supported robots](#supported-robots)
  - [Real robots](#real-robots)
    - [Robot bodies](#robot-bodies)
    - [End effectors](#end-effectors)
    - [Cameras](#cameras)
    - [Mobile bases](#mobile-bases)
  - [Sim robots](#sim-robots)
- [Quick start](#quick-start)
  - [Simulation](#simulation)
  - [Real robot: Unitree G1](#real-robot-unitree-g1)
  - [Several robots as one](#several-robots-as-one)
- [Add a new robot](#add-a-new-robot)
- [Contact](#contact)
- [Citation](#citation)
- [License](#license)

</details>

<!-- /toc -->

## Introduction

hiRobot gives models and agents a single, consistent way to observe and command
a robot — **humanoid or arm, in simulation or on real hardware** — regardless of
the vendor SDK underneath. It standardizes the embodiment I/O (joints, optional
floating base, optional end-effectors, cameras, control rate) and the robot-driver
contract, so control code written once runs across different robots without
rewrites.

Robots come in many flavors: ~30 DoF humanoids with a floating base (Unitree,
Fourier GRx, AgiBot X1), dual arms on a wheeled base (AgiBot G1) and fixed-base
arms with Cartesian control (Franka, UR).
Wiring a model or agent to each one usually means bespoke glue per robot and per
SDK. hiRobot standardizes that glue:

- **One interface, many robots** — write control logic against the `Robot`
  contract and run it on any robot that provides a driver. Each robot's joints
  (by name), base, and end-effectors are described in config, so the same
  policy can target different robots. A `RobotGroup` drives several complete
  robots (different vendors, each with its own hands, gate and whole-body
  control) plus scene cameras as one `Robot`, placed in a shared world frame,
  so a bimanual VLA or an agent controls them without new code.
- **Sim and real, symmetric** — simulation and hardware are just different
  drivers behind the same contract, selected by one config field. The MuJoCo
  robot runs on its own for debugging without hardware, or next to a real robot
  as a twin that mirrors its measured state (`mirror`) or rehearses a motion from
  its current pose (`shadow`). Simulated benchmark environments plug in as
  drivers behind the same contract, so code evaluated in simulation runs
  unchanged on the robot.
- **VLA agnostic** — hiRobot's `Observation` / `ActionChunk` types are what a
  VLA reads and produces, whichever model it is. Swapping one VLA for another
  (GR00T, π0.5, StarVLA) changes the hiVLA server hiAgent points at, not the
  robot code.
- **Agent friendly** — the interface is easy for an agent to call as tools:
  `Robot.action_modes` lists the control modes a driver accepts, `get_status`
  reports battery, temperatures and joint faults, robot skills (walk, gestures)
  come with a description and a JSON Schema for their arguments, and
  `SafetyGate` checks every command, so a refused action raises `SafetyError`
  instead of reaching the motors. [docs/api.md](docs/api.md) documents every
  call with its arguments and failure modes.
- **Step-by-step deployment docs** — real-robot deployment involves many
  details a newcomer can't guess, so each supported robot gets a setup guide
  covering SDK install, network setup, power-on and safety, connection
  parameters, the first control loop, and common pitfalls.
- **Decoupled by design** — each part has its own contract and registry
  (body, hand, camera, audio), and each driver lives in its own
  `drivers/<part>/<SDK vendor>/<SDK or protocol>/` directory with a manifest
  of the SDK it needs, a fake SDK and a shared contract test. Configs are
  composed from one file per part with `defaults:`, and controllers such as
  whole-body control plug in as `layers:` from their own packages. A new model
  on a supported SDK is configs only; a new SDK is one driver directory and its
  tests, with no change to the core.

> [!NOTE]
> **Companion projects.** [hiWBC](https://github.com/humanoidintelligence/hiWBC)
> handles whole-body balance and locomotion beneath the robot interface,
> [hiVLA](https://github.com/humanoidintelligence/hiVLA) (the motor cortex) serves
> VLA models over the network, and
> [hiAgent](https://github.com/humanoidintelligence/hiAgent), the agentic operating
> layer, drives the robot from above. The text below uses them as examples;
> hiRobot works on its own and imports none of them.

> [!WARNING]
> **Status — early development.** APIs are unstable and subject to change.
>
> **Safety on real robots.** A robot under software control can injure people and
> damage itself. Read this repository carefully before running it on hardware,
> including the [setup guide](docs/setup/README.md) for your robot.
> Debug in simulation until it works, then move to the real robot step by step:
> read-only first, then the robot hung on a gantry at low gains, with an e-stop
> within reach and nobody inside the robot's reach.

## Architecture

![hiRobot architecture. Inside hiRobot: make_robot reads a config into an Embodiment and builds a stack, outermost first: the Robot contract (Observation / Action), the SafetyGate inside send_action, RobotAssembly (cameras, hands, audio), optional layers, an optional TwinRobot MuJoCo twin, and the drivers (real robots, simulators, hands, cameras, audio) that talk to vendor SDKs and simulators. Utilities beside it: RobotGroup, collision, DofMap, Kinematics, transforms. Around it: hiVLA and hiAgent above, exchanging observations and action chunks over the network, with actions going down to the Robot and observations coming back up; hiWBC plugs in as a layer. All companions are optional.](assets/architecture.png)

hiRobot owns the robot I/O. `make_robot` reads a config (parts in `configs/`
composed into an `Embodiment`) and builds a stack of `Robot`s, drawn outermost
first: `RobotAssembly` attaches cameras, dexterous hands and audio; optional
layers (plugins registered with `register_layer`) wrap the body; an optional
`TwinRobot` pairs a real robot with a MuJoCo twin; at the bottom a driver
talks to the vendor SDK or simulator. Every command leaves through
`Robot.send_action`, which runs the `SafetyGate` before the driver touches the
hardware and raises `SafetyError` on a refusal, so there is one enforced place
for safety checks that no caller can skip.

The companions use the same `Robot` from outside. hiWBC registers a layer
(`layers: [{type: wbc, ...}]` in a config) that sits on the humanoid's body and
turns base velocity, motion clips and motion latents into balanced joint
targets every tick; the layer is itself a `Robot`, so callers above it do not
change. hiAgent runs the control loop and calls the `Robot` in-process: it reads
an `Observation`, sends it to a model served by hiVLA, and sends the returned
action chunk to the robot one `Action` at a time.

Core concepts:

| Concept | Responsibility |
| --- | --- |
| `Embodiment` | What a robot looks like to a policy: joints (by name, in the SDK's order), `floating_base`, end-effectors, cameras, microphones / speakers, `control_hz`. Loaded from the `embodiment:` section of a config. |
| `Observation` / `Action` / `ActionChunk` | Typed I/O. Joint space (universal) + optional base (humanoids) + optional task space (arms) + images. `ControlMode` selects joint, Cartesian, base velocity or motion targets; an `ActionChunk` is several predicted steps. |
| `Robot` | The contract every driver, simulator and wrapper implements: `connect` / `reset` / `get_observation` / `send_action` / `hold` / `set_gains` / `get_status` / `disconnect`, plus `action_modes` (the modes it accepts) and robot skills (motions the robot's own controller runs). Drivers implement `_send_action`; `send_action` runs the gate first. [Details](docs/api.md#robot). |
| `SafetyGate` / `SafetyError` | Checks shape, DoF count and finiteness of every action inside `send_action` (and optionally clamps or refuses out-of-range targets); a refusal raises `SafetyError`. Set by the config's `safety:` section. |
| `make_robot` / registries | Builds the stack from a config name or path. Drivers and layers are found by name in registries (`register_body`, `register_end_effector`, `register_camera`, `register_audio`, `register_layer`); packages add their own through the `hirobot.drivers` entry point. |
| `RobotAssembly` / `TwinRobot` | Wrappers `make_robot` adds: devices attached to a body; a real robot paired with a MuJoCo twin (`mirror` / `shadow`). |
| `RobotGroup` | Several complete robots plus scene cameras as one `Robot`, in a shared world frame. Built from a `group:` config. |
| `DofMap` / `Kinematics` | Reorder joint vectors by name between layouts; forward kinematics of an MJCF model. |

## Repository structure

<details>
<summary><b>Show the tree</b></summary>

```
hiRobot/
├── src/hirobot/
│   ├── embodiment.py        # Embodiment: joints, base, end-effectors, cameras, audio
│   ├── types.py             # Observation / Action / ActionChunk, ControlMode, RobotStatus
│   ├── robot.py             # Robot driver contract
│   ├── robot_skills.py      # robot skills (walk, gestures): specs, argument checks
│   ├── end_effector.py camera.py audio.py   # end-effector / camera / audio contracts and registries
│   ├── config.py            # config names, defaults: composition, merge rules
│   ├── registry.py          # body / layer registries, entry-point plugins (`hirobot.drivers`)
│   ├── safety.py            # SafetyGate: checks every action in send_action
│   ├── assembly.py          # RobotAssembly: body + attached cameras, end-effectors, audio
│   ├── twin.py              # TwinRobot: real robot + MuJoCo twin (mirror / shadow)
│   ├── group.py             # RobotGroup: several robots + scene cameras as one Robot
│   ├── collision.py         # distance check between group members
│   ├── dof.py               # DofMap: reorder joint vectors by name
│   ├── kinematics.py        # forward kinematics of an MJCF model
│   ├── transforms.py        # pose math on [x, y, z, qx, qy, qz, qw]
│   ├── sdk.py               # require_sdk: check a driver's SDK against its manifest.yaml
│   ├── cli.py               # `hirobot info`, `hirobot configs`, `hirobot sim`
│   ├── configs/             # built-in configs (installed with the package)
│   │   ├── body/ end_effector/ camera/ audio/   # one file per part, by vendor
│   │   ├── robots/          # whole robots, composed from parts with defaults:
│   │   ├── sim/mujoco/      # overlays that put a whole robot in MuJoCo
│   │   └── groups/          # robot groups (franka_ur5e_dual)
│   └── drivers/             # <part>/<SDK vendor>/<SDK or protocol>/{driver.py, manifest.yaml}
│       ├── body/            # unitree/sdk2, fourier/grx, agibot/aimrt, franka/libfranka, ur/rtde
│       ├── end_effector/    # hands: unitree/sdk2_handcmd (Dex3-1), unitree/sdk2_motorcmds, inspire/sdkpy
│       ├── camera/          # unitree/sdk2_videohub, unitree/image_server; generic/{video_file,array}
│       ├── audio/           # unitree/sdk2_voice; generic/{wav,sounddevice}
│       ├── sim/             # toy/ (`toy`), mujoco/ (`mujoco`: a robot of its own, also the twin)
│       └── _sdk/            # code shared inside one SDK (Unitree DDS init, once per process)
├── examples/                # configs to copy: real robot, read_only, shadow twin, groups
├── scripts/
│   ├── unitree_setup.py          # status / hold / move / restore on a Unitree robot
│   ├── unitree_estop.py          # software e-stop (damping on Ctrl-C)
│   ├── dex3_setup.py             # status / wiggle on the G1 Dex3 hands (body never commanded)
│   ├── inspire_serial_to_dds.py  # H1 Inspire hands: RS485 serial <-> DDS bridge, runs on PC2
│   ├── inspire_hand_demo.py      # open the Inspire hands slowly, then return (body untouched)
│   └── camera_image_server.py    # serve a camera as JPEG over ZeroMQ (image_server format)
├── docs/
│   ├── api.md               # public API reference (api.zh-CN.md: Chinese)
│   └── setup/               # per-robot hardware setup guides (<robot>.zh-CN.md: Chinese)
├── tests/                   # pytest suite; simulators and SDKs are faked, no GPU needed
│   ├── drivers/             # mirrors src/hirobot/drivers/: fake_sdk.py + test_contract.py per SDK
│   └── data/                # expanded_configs.json: snapshot of every built-in robot config
└── assets/                  # README images
```

</details>

## Supported robots

Simulators and hardware are different drivers behind the same `Robot` contract,
so one control stack moves from sim to real by swapping the driver. Simulator
and vendor SDKs are imported lazily inside each driver; the core never depends
on them.

### Real robots

Each robot has an embodiment config and a vendor driver. The shipped configs
default to the `toy` driver; set `body.driver` once the robot is set up per its
setup guide. ✅ marks drivers that have run on the real robot. A date ("By Nov 11",
"By Nov 15") marks the rest: a driver covered by tests but not yet run on
hardware, or a robot with no driver yet, planned to be done by that date (2026).

#### Robot bodies

| Robot | DoF | Configs | Driver | Vendor SDK | Setup guide | Hardware-verified |
| --- | --- | --- | --- | --- | --- | :---: |
| Unitree G1 | 29 | `unitree_g1` | `unitree_sdk2` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md) | ✅ |
| Unitree H1 | 19 | `unitree_h1` | `unitree_sdk2` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md) | ✅ |
| Unitree H1-2 | 27 | `unitree_h1_2` | `unitree_sdk2` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md) | By Nov 15 |
| Unitree H2 | 31 | `unitree_h2` | `unitree_sdk2` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md) | By Nov 15 |
| Fourier GR1T1 | 32 | `fourier_gr1t1` | `fourier_grx` | fourier-grx-client | [fourier.md](docs/setup/fourier.md) | By Nov 15 |
| Fourier GR1T2 | 32 | `fourier_gr1t2` | `fourier_grx` | fourier-grx-client | [fourier.md](docs/setup/fourier.md) | By Nov 11 |
| Fourier GR2 | 29 | — | — | fourier_aurora_client | — | By Nov 15 |
| AgiBot G1 | 18 | `agibot_g1` | `agibot_gdk` | GDK topics (cosine_bus) | [agibot_g1.md](docs/setup/agibot_g1.md) | ✅ |
| AgiBot G2 | 22 | — | — | agibot_gdk (GDK 2.1) | — | By Nov 15 |
| AgiBot X1 | 29 | `agibot_x1` | `agibot_aimrt` | ROS2 bridge to AimRT | [agibot_x1.md](docs/setup/agibot_x1.md) | By Nov 15 |
| AgileX ALOHA | 12 | — | — | pyAgxArm | — | By Nov 11 |
| AgileX PiPER | 6 | — | — | pyAgxArm | — | By Nov 11 |
| Franka FR3 | 7 | `franka_fr3` | `franka_libfranka` | pylibfranka | [franka.md](docs/setup/franka.md) | By Nov 11 |
| UR5e | 6 | `ur_ur5e` | `ur_rtde` | ur_rtde | [ur.md](docs/setup/ur.md) | By Nov 11 |
| WidowX 250S | 6 | — | — | dynamixel_sdk | — | By Nov 15 |

Joint names and ordering in each config come from the vendor SDK, so they line
up 1:1 with the hardware. Robot differences live in config and hardware
differences in the driver: the four Unitree configs share one `unitree_sdk2` driver,
and both Fourier configs share the `fourier_grx` driver.

#### End effectors

A dexterous hand, or a gripper with an SDK of its own, has its own end-effector
driver, attached to a body through the `driver:` key of an `end_effectors` entry
in the config; each hand config below is a body plus both hands. A hand lists its
joints and is commanded through `joint_target`. A gripper lists none and is read
and commanded through `Observation.gripper` / `Action.gripper` (0 closed, 1 open),
in any control mode. A gripper the body's own SDK drives has no `driver:` key, and
the body driver handles `gripper` itself.

| End effector | DoF each | Mounts on | Configs | Driver | Interface | Vendor SDK | Setup guide | Hardware-verified |
| --- | --- | --- | --- | --- | --- | --- | --- | :---: |
| Unitree Dex3-1 | 7 | Unitree G1 | `unitree_g1_dex3` | `unitree_handcmd` | `joints` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md#dex3-hands-g1) | ✅ |
| Inspire RH56DFX | 6 | Unitree H1 | `unitree_h1_inspire` | `unitree_motorcmds` (h1_inspire_service 1.0.1) or `inspire_sdkpy` | `joints` | unitree_sdk2py | [unitree.md](docs/setup/unitree.md#inspire-hands-h1) | ✅ |
| AgiBot Omnipicker gripper (×2) | 1 | AgiBot G1 | `agibot_g1` | `agibot_gdk` (body driver) | `gripper` | GDK topics (cosine_bus) | [agibot_g1.md](docs/setup/agibot_g1.md#4-observation-and-action) | ✅ |
| Robotiq 2F-85 | 1 | — | — | — | `gripper` | pymodbus or UR URCap socket | — | By Nov 15 |

#### Cameras

A camera is a `cameras` entry in the config with a `driver:`; with a real body
driver, each frame lands in `Observation.images[<name>]`. Sources only read, so
they also run in `read_only` and `sim: shadow`. Stand-ins for tests and
simulation (`video_file`, `array`) are in [docs/api.md](docs/api.md#extension-points-cameras-end-effectors-and-audio).

| Camera | On | Configs | Source | Transport | Setup guide | Hardware-verified |
| --- | --- | --- | --- | --- | --- | :---: |
| RealSense D435i (head) | Unitree G1 | `unitree_g1`, `unitree_g1_dex3` | `unitree_videohub` | unitree_sdk2py videohub (`videohub_pc4` on PC2) | [unitree.md](docs/setup/unitree.md#head-camera-g1) | ✅ |
| RealSense D435i (head) | Unitree H1 | `unitree_h1`, `unitree_h1_inspire` | `unitree_image_server` | JPEG over ZeroMQ (Unitree `image_server`, or `scripts/camera_image_server.py`) | [unitree.md](docs/setup/unitree.md#head-camera-h1) | ✅ |
| RealSense D457 (head) | AgiBot G1 | `agibot_g1` | `agibot_cosine` | cosine_bus topics on the robot's computer | [agibot_g1.md](docs/setup/agibot_g1.md#3-configuration) | ✅ |
| RealSense D405 (wrist, ×2) | AgiBot G1 | `agibot_g1` | `agibot_cosine` | cosine_bus topics on the robot's computer | [agibot_g1.md](docs/setup/agibot_g1.md#3-configuration) | ✅ |

#### Mobile bases

A wheeled base is driven by its body's driver and commanded with
`BASE_VELOCITY` (`[vx, vy, wz]`, m/s and rad/s); its odometer fills
`Observation.base_position` / `base_orientation`.

| Base | Under | Configs | Driver | Transport | Commands | Setup guide | Hardware-verified |
| --- | --- | --- | --- | --- | --- | --- | :---: |
| AgiBot G1 chassis | AgiBot G1 | `agibot_g1` with `chassis_host` | `agibot_gdk` (body driver) | HTTP + WebSocket to the chassis controller box | `[vx, 0, wz]`, in soft-manual mode only | [agibot_g1.md](docs/setup/agibot_g1.md#2-power-on-and-safety) | ✅ |

### Sim robots

A MuJoCo robot runs the same `Robot` interface, safety gate and `layers:` as
the real one, with names, joint order and cameras taken from the real robot's
config. It is used in three ways:

| Use | How | Real robot |
| --- | --- | --- |
| On its own | A `*_mujoco` config (`unitree_g1_mujoco`, `unitree_g1_mujoco_standing`, `unitree_g1_dex3_mujoco`), or your config plus a `sim/mujoco/...` overlay | None |
| `mirror` | `sim: {mode: mirror}` in a real robot's config: the real robot is commanded and a MuJoCo viewer shows its measured state | Commanded |
| `shadow` | `sim: {mode: shadow}`: the real robot is opened read-only and every action runs on the twin, starting from the robot's current pose, so a motion is rehearsed before it is sent | Read-only |

The models are not shipped; the G1 MJCF is fetched from Unitree's
[unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)
([commands](docs/setup/unitree.md#5-mujoco-g1-optional)). The dependency-free
`toy` driver, which every shipped config defaults to, integrates joint targets
in-process for tests.

## Quick start

The core install is light (numpy, pydantic, pyyaml); extras add what a given
driver needs:

```bash
git clone https://github.com/humanoidintelligence/hiRobot.git && cd hiRobot
pip install -e .                 # core: interface, toy driver, safety gate
# extras: sim (MuJoCo robot / twin) · camera · video · audio · dev
```

Inspect the embodiment a config describes (no robot, no GPU):

```bash
hirobot info --config unitree_g1   # a humanoid
hirobot info --config ur_ur5e      # a fixed-base arm
```

### Simulation

Debug control code on a MuJoCo G1 first. Install the extra and fetch the G1
model ([commands](docs/setup/unitree.md#5-mujoco-g1-optional)), then open it in
the viewer:

```bash
pip install -e '.[sim]'
hirobot sim unitree_g1_mujoco --viewer
```

The basic control loop: read an observation, send an action, and hold at the
end. This one bends the right elbow to 1 rad over 2 s:

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("unitree_g1_mujoco")   # pelvis welded, like the robot on its gantry
robot.connect()
robot.reset()
elbow = robot.embodiment.joint_names.index("right_elbow")

for step in range(100):                   # 2 s at control_hz = 50
    obs = robot.get_observation()         # joint_pos, base IMU, images["head"]
    target = obs.joint_pos.copy()
    target[elbow] = min(target[elbow] + 0.01, 1.0)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
robot.hold()
robot.disconnect()
```

A simulator steps once per `send_action`, so the loop needs no timer; on a real
robot, pace it at `robot.embodiment.control_hz`. The same code runs on the real
G1 once the config changes.

### Real robot: Unitree G1

From a fresh clone to a first motion on a G1: lift the right forearm to
horizontal and back. The full procedure, with what to check at each step and
the results of a real run, is in
[the Unitree setup guide](docs/setup/unitree.md#6-first-motion-step-by-step-g1-elbow).

**1. Install the SDK and connect.** Install
[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python),
plug the robot in over Ethernet, and put the host NIC on `192.168.123.x`
([network](docs/setup/unitree.md#2-network)). For the MuJoCo twin, also
`pip install -e '.[sim]'` and fetch the G1 model
([twin setup](docs/setup/unitree.md#twin-next-to-the-real-robot)).

**2. Write a local config.** Start it with `defaults: [unitree_g1]`, so the
joints, camera and audio come from the built-in config, then set
`body.driver: unitree_sdk2` and per-joint gains, and optionally add a `sim:` section
for the twin. [`examples/unitree_g1_real.yaml`](examples/unitree_g1_real.yaml) has
working G1 gains; [`examples/`](examples/README.md) also has read-only and shadow
versions.

**3. Read the robot (read-only).** Joint angles, IMU, temperatures, and whether
a built-in motion service owns the command topic:

```bash
python scripts/unitree_setup.py status --config my_g1.yaml --iface eth0
python scripts/unitree_estop.py --iface eth0     # second terminal: software e-stop
```

**4. Rehearse in the twin (read-only).** The real robot is opened read-only and
the move runs in MuJoCo from its current pose:

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --sim shadow
```

**5. Run it on the robot.** Hang the robot with its feet off the ground and keep
the e-stop in hand. `--sim mirror` shows the real robot in MuJoCo while it moves;
leave it out to run without MuJoCo:

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --release-motion-mode --sim mirror
```

The move aborts to damping if any joint strays more than 0.3 rad, and always
ends in damping. `restore` hands control back to the built-in service.

Other robots follow their own
guides: [overview](docs/setup/README.md) ·
[Fourier GR1T1 / GR1T2](docs/setup/fourier.md) ·
[AgiBot G1](docs/setup/agibot_g1.md) · [AgiBot X1](docs/setup/agibot_x1.md) · [Franka FR3](docs/setup/franka.md) ·
[Universal Robots UR5e](docs/setup/ur.md).

### Several robots as one

A config with a `group:` section builds a `RobotGroup`: each member is a whole
single-robot config, and the group is itself a `Robot`, so the loop above
drives it unchanged. The built-in `groups/franka_ur5e_dual` runs on toy drivers:

```python
from hirobot import Action, ControlMode, make_robot

group = make_robot("groups/franka_ur5e_dual")   # Franka FR3 + UR5e + a scene camera
group.connect()
group.reset()
obs = group.get_observation()                    # joints concatenated: left/..., right/...
left = group.member("left").get_observation()    # one member, in its own names
group.send_action(Action(members={               # one sub-action per member, each in its own mode
    "left": Action(mode=ControlMode.POSITION, joint_target=left.joint_pos + 0.1),
}))                                              # "right" is not named and keeps its target
group.disconnect()
```

For hardware, each member gets its driver and address under
`group.members.<name>.overrides`
([`examples/franka_ur5e_dual_real.yaml`](examples/franka_ur5e_dual_real.yaml)).
Every part of an action is checked by its member's gate before anything is
sent, and a member that fails puts the whole group on hold. Flat actions,
world frames and naming rules are in [docs/api.md](docs/api.md#robotgroup).

## Add a new robot

> [!TIP]
> **Pull requests are welcome.** A new robot, hand, camera or SDK version, a
> hardware check of a driver not yet marked ✅, or a fix to a setup guide each
> brings robot control a step closer to one shared interface. Before opening a
> pull request, read [CONTRIBUTING.md](CONTRIBUTING.md): what each kind of change
> needs, hardware checks and reviewers.

A robot is supported in two layers: **configs** (its joint layout and parts) and
a **driver** (the hardware link). Code is organized by how it talks to the
hardware: `drivers/<part>/<SDK vendor>/<SDK or protocol>/`, named after the SDK,
not the product. Models live in configs: `configs/<part>/<vendor>/<model>.yaml`
says which driver to use. One SDK usually covers many models, so most new
robots need only configs.

| Case | What to add | Example |
|---|---|---|
| New model, same SDK and protocol | Configs only: `body/<vendor>/<model>.yaml` and `robots/<name>.yaml` | Unitree H2D: `body/unitree/h2d.yaml` |
| New version of a supported SDK | A `verified` entry in the driver's `manifest.yaml` once it ran on hardware; a fake-SDK test if behavior changed | G1 + Dex3 on `unitree_sdk2py` 1.1 |
| New SDK or protocol | A new SDK directory with `driver.py`, `manifest.yaml`, a new registered name, and `tests/drivers/.../{fake_sdk,test_contract}.py` | Fourier Aurora, AgiBot OmniHand |
| SDK generation break (message layout, control or gain semantics change) | A new directory under the same vendor (`sdk3/`) and a new name (`unitree_sdk3`) | |

Registered names are `<vendor>_<SDK or protocol>` (`unitree_sdk2`, `fourier_grx`).
A released name never changes meaning, so updating hiRobot cannot quietly swap
the driver a real robot runs.

**Configs.** Put each part in its own file under `src/hirobot/configs/`
(`body/`, `hand/`, `camera/`, `audio/`) and compose the whole robot in
`robots/<name>.yaml` with `defaults:`. List joints by name in the SDK's order and
set `floating_base`, end-effectors, cameras and `control_hz`. Check the result
with `hirobot info --config <name> --resolved`.

**A driver.** Subclass `Robot` (or `EndEffectorDriver`, `CameraDriver`,
`MicrophoneDriver` / `SpeakerDriver`), map `Observation` / `Action` onto the SDK's
reads and commands, and register it (`@register_body("<vendor>_<sdk>")`). Import
the SDK inside `connect()` after `require_sdk(__package__)`, so `hirobot` imports
without it and a missing or old SDK gives a clear error. `manifest.yaml` lists
the SDK (`distribution`, `minimum`, `install`) and starts with `verified: []`.
Report `base_orientation` as `[x, y, z, w]`.

**Tests.** Write a fake of the SDK calls the driver makes in
`tests/drivers/<part>/<vendor>/<sdk>/fake_sdk.py`, and run the shared checks
from `tests/drivers/contract.py` against it in `test_contract.py`
(`check_robot`, `check_hand`, `check_camera`, `check_microphone`,
`check_speaker`). They cover the registry, the manifest, connect / read /
command / disconnect, per-joint shapes, the quaternion and the safety gate.
`tests/drivers/test_coverage.py` fails while a registered driver has none.
Hardware runs cannot go in CI: record them as `verified` entries
(model, SDK version, date).

**Outside the repository.** A separate package can register drivers through the
`hirobot.drivers` entry-point group and release on its own SDK's schedule;
user configs stay the same.

## Contact

- [Peng Li](https://artpli.github.io): pengli.ds [at] gmail.com

## Citation

If you use hiRobot in your research, please cite it:

```bibtex
@misc{hirobot,
  title  = {hiRobot: A Unified Interface Layer for Controlling Diverse Robots from VLA Models and Agents},
  author = {Li, Peng and Zhang, Yifei and Zhuang, Zihan and Dong, Yi and Gui, Tao and Qiu, Xipeng},
  year   = {2026},
  url    = {https://github.com/humanoidintelligence/hiRobot}
}
```

## License

[Apache License 2.0](LICENSE). See [LICENSE](LICENSE) for the full terms.
