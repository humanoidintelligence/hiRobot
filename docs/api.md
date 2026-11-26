# hiRobot API reference

English | [简体中文](./api.zh-CN.md)

This page lists the interfaces hiRobot exposes to users: what you import from
`hirobot`, what a config file may contain, the CLI, and the extension points for
adding a driver, camera, or end-effector (hand or gripper). Everything listed under
[Top-level package](#top-level-package) is importable as `from hirobot import ...`.

Conventions used throughout:

- Arrays are `numpy.ndarray`. Angles are radians, angular rates rad/s, torques Nm.
- Joint vectors (`joint_pos`, `joint_target`, `kp`, ...) follow the order of
  `Embodiment.joints`. Use [`DofMap`](#dofmap) when another ordering is needed.
- Quaternions are `[x, y, z, w]`. Poses are 7-vectors `[x, y, z, qx, qy, qz, qw]`.

## Contents

- [Robot at a glance](#robot-at-a-glance)
- [Quick example](#quick-example)
- [Top-level package](#top-level-package)
- [Building robots: make_robot and the driver registry](#building-robots-make_robot-and-the-driver-registry)
- [Robot](#robot)
- [Embodiment](#embodiment)
- [Observation, Action, ActionChunk, ControlMode](#observation-action-actionchunk-controlmode)
- [Skills](#skills)
- [Onboard skills](#onboard-skills)
- [Command arbiter](#command-arbiter)
- [SafetyGate](#safetygate)
- [RobotAssembly](#robotassembly)
- [Control loops](#control-loops)
- [DofMap](#dofmap)
- [Features](#features)
- [Recording](#recording)
- [Kinematics](#kinematics)
- [Simulation assets](#simulation-assets)
- [Config file reference](#config-file-reference)
- [CLI](#cli)
- [MCP server](#mcp-server)
- [Writing a driver or a layer (SPI)](#writing-a-driver-or-a-layer-spi)
- [Extension points: cameras, end-effectors and audio](#extension-points-cameras-end-effectors-and-audio)

## Robot at a glance

Everything a `Robot` has, by what it is for. Drivers, simulators, assemblies
and layers all have every member below; signatures and behaviour are in
[Robot](#robot).

| For | Members | See |
|---|---|---|
| What the robot is | `embodiment`, `action_modes`, `motion_spaces`, `direct_joints`, `cartesian_targets`, `accepts_parts`, `simulated` | [Embodiment](#embodiment), [ControlMode](#controlmode) |
| Attached devices | `cameras`, `microphones`, `speakers` (by name) | [Extension points](#extension-points-cameras-end-effectors-and-audio) |
| Connect and reset | `connect()`, `reset(seed=None)`, `disconnect()` | [Robot](#robot) |
| Read | `get_observation() -> Observation`, `get_status() -> RobotStatus`, `now()`, `fault` | [Observation](#observation), [RobotStatus](#robotstatus) |
| Command | `send_action(action)`, `check_action(action) -> Action`, `hold()`, `set_gains(kp, kd)`, `command_timeout_s` | [Action](#action) |
| Stop | `estop()` | [Robot](#robot) |
| Share the command stream | `claim(name, ...) -> CommandLease`, `command_events()` | [Command arbiter](#command-arbiter) |
| Watch (record, monitor) | `add_listener(on_observation=None, on_command=None) -> ListenerHandle` | [Listeners](#listeners), [Features](#features) |
| Skills | `list_robot_skills()`, `run_robot_skill(name, args=None)`, `stop_robot_skill()`, `robot_skill_running` | [Skills](#skills) |
| Skills by kind | Onboard skills: `has_onboard_skills`, `list_onboard_skills()`, `run_onboard_skill(name, args=None)`, `stop_onboard_skill()`. Layer skills: `list_layer_skills()`, `run_layer_skill(name, args=None)`, `stop_layer_skill()`, `layer_skill_running` | [Onboard skills](#onboard-skills), [Layers](#layers) |
| Safety and episode outcome | `safety` (the attached `SafetyGate`), `success`, `done`, `reward` | [SafetyGate](#safetygate), [Control loops](#control-loops) |

## Quick example

```python
import numpy as np
from hirobot import Action, ControlMode, make_robot

robot = make_robot("examples/humanoid.yaml")  # from the repository root; body.driver: toy
robot.connect()
try:
    robot.reset()
    for _ in range(100):                             # hold every joint at zero
        obs = robot.get_observation()
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.zeros(robot.embodiment.num_dofs)))
finally:
    robot.disconnect()
```

## Top-level package

| Name | Kind | Module | Purpose |
|---|---|---|---|
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `CameraIntrinsics`, `CameraInfo`, `AudioDevice`, `ModelRef`, `EmbodimentKind` | pydantic models / enum | `hirobot.core.embodiment` | Declarative robot I/O contract |
| `Observation`, `Action`, `ActionChunk`, `ControlMode`, `MotionClip`, `JOINT_MODES`, `MOTION_SPACES` | pydantic models / enum / tuples | `hirobot.core.types` | Typed I/O between robot and policy |
| `RobotStatus`, `BatteryState`, `CommandProgress` | pydantic models | `hirobot.core.types` | What `get_status()` returns (see [`RobotStatus`](#robotstatus)) |
| `Robot` | abstract class | `hirobot.core.robot` | Driver contract |
| `RobotAssembly` | `Robot` | `hirobot.assembly` | A robot assembled from a body driver plus attached cameras, end-effectors (hands, grippers), microphones and speakers; or, with child parts, several complete robots driven as one (see [RobotAssembly](#robotassembly)) |
| `Layer` | `Robot` | `hirobot.control.layer` | Base class for layers: wraps `inner` and forwards to it (see [Layers](#layers)) |
| `RobotFaultError` | exception | `hirobot.core.robot` | Raised by `send_action` while the robot is in a fault state |
| `RobotStoppedError` | exception | `hirobot.core.robot` | `RobotFaultError` subclass: a long call (`connect`, `reset`) gave up because `estop()` ran |
| `register_body`, `get_body`, `available_bodies` | functions | `hirobot.loading.registry` | Name-based driver registry |
| `register_layer`, `get_layer`, `available_layers` | functions | `hirobot.loading.registry` | Registry of layers stacked by `layers:` (see [Layers](#layers)) |
| `load_embodiment`, `make_robot` | functions | `hirobot` | Build an embodiment / robot from a config name, file or dict |
| `model_path`, `available_configs` | functions | `hirobot` | The model file of a config's one body; the built-in config names |
| `require_sdk` | function | `hirobot.loading.sdk` | Check a driver's vendor SDK against its `manifest.yaml` |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic models / exception | `hirobot.core.robot_skills` | Describe, start and refuse onboard skills (walk, gestures) |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | class / dataclass / exception | `hirobot.control.safety` | Check every action before it reaches hardware |
| `DofMap` | class | `hirobot.core.dof` | Reorder joint vectors by name |
| `CommandRecord`, `ListenerHandle` | frozen dataclass / class | `hirobot.control.arbiter` | What an `on_command` listener gets; what `add_listener` returns (see [Listeners](#listeners)) |
| `observation_features`, `flatten_observation`, `action_features`, `flatten_action`, `image_features` | functions | `hirobot.core.features` | `Observation` / `Action` as named vectors (see [Features](#features)) |
| `pose_compose`, `pose_inverse`, `pose_from_matrix`, `pose_from_axis_angle`, ... | functions | `hirobot.geometry.transforms` | Pose math on 7-vector poses `[x, y, z, qx, qy, qz, qw]` (not re-exported from `hirobot`) |
| `Kinematics`, `BodyPoses` | classes | `hirobot.geometry.kinematics` | Forward kinematics of an MJCF model (see [Kinematics](#kinematics); not re-exported from `hirobot`) |
| `__version__` | str | `hirobot` | Package version |

The interfaces and registries for cameras, end-effectors and audio are in `hirobot.devices.camera`,
`hirobot.devices.end_effector` and `hirobot.devices.audio`; the drivers themselves live under `hirobot.drivers`
(`drivers/<part>/<SDK vendor>/<SDK or protocol>/`). Both are covered in [Building robots](#building-robots-make_robot-and-the-driver-registry)
and [Extension points](#extension-points-cameras-end-effectors-and-audio).

## Building robots: make_robot and the driver registry

### `load_embodiment(config) -> Embodiment`

The `Embodiment` a robot config (`assembly:`) or a device file (`device:`)
describes: a built-in name (`"unitree_g1"`, the same as `"robots/unitree_g1"`),
a file path, or an already-loaded dict, after `defaults:` are expanded (see
[Config file reference](#config-file-reference)). For an assembly with parts the
embodiment is the merged tree, every part's names under its `<part>/` prefix.
Nothing is connected or built. Raises `ValueError` if the config is neither.

### `model_path(config) -> Optional[ModelRef]`

The model file of the config's one body, without building the robot: the same
as `load_embodiment(config).model`, except for the body count. A config with no
body or more than one raises `ValueError` (the message lists the body paths).
`None` means one body whose device file has no `model:`. The file is not checked
to exist. A config that does not load raises what `load_embodiment` raises
(`ValueError`, `FileNotFoundError`), so a caller that needs to tell those apart
calls `load_embodiment` first:

```python
import os
import hirobot

try:
    hirobot.load_embodiment(config)
except Exception as e:                # the config does not load
    ...
try:
    ref = hirobot.model_path(config)
except ValueError as e:               # no body, or more than one
    ...
if ref is None:                       # one body without model:
    ...
elif ref.kind == "mjcf" and os.path.isfile(ref.path):
    ...
```

### `available_configs(group=None) -> List[str]`

Names of the built-in configs, sorted: `robots/unitree_g1`, `body/unitree/g1`, ...
`group` (`robots`, `body`, `end_effector`, `camera`, `microphone`, `speaker`)
limits the list to one directory. What `hirobot configs` prints.

### `make_robot(config, overrides=None, sim=None) -> Robot`

Builds the configured robot. `config` is a built-in name, a file path, or a dict:
a robot config (`assembly:`) or a device file alone (`device:`). The config is
loaded and built as a tree, one robot per node, the drivers looked up in the
registry (built-in drivers and `hirobot.drivers` entry-point plugins). Steps, in
order:

1. Expand `defaults:` and load the tree. Each node's own devices (`body`, with
   its hands, cameras and audio) form its own robot; each device's `driver:` is
   looked up in the registry (`body` defaults to `toy`).
2. With a real body and end-effectors that have a `driver:`, build one
   end-effector driver for each and give the body driver only the body joints
   (and none of the grippers those drivers own). Instantiate the body as
   `driver_cls(embodiment, **driver_kwargs)`; the other `driver:` keys are
   constructor arguments.
3. Build a camera driver for each camera with a `driver:` and an audio driver
   for each microphone / speaker with a `driver:`. With a simulated body, only
   stand-ins (`simulated = True`, e.g. `array`, `video_file`, `wav_mic`) are
   built, so the config runs without the robot.
4. Join a node's devices (body, end-effectors, cameras, audio) in one
   `RobotAssembly` when there is a device to attach. A node with child parts
   becomes a `RobotAssembly` holding those parts (and the node's own body, if
   any); nodes nest.
5. Wrap the root in one `SafetyGate`, with per-joint limits merged from every
   `safety:` in the tree (the joint-count and non-finite checks always apply);
   `safety: false` detaches it.
6. If `layers:` is set, wrap the robot in each layer in order (see
   [Layers](#layers)). A layer with `part: <path>` wraps only that part of an
   assembly (`hirobot.assembly.part_layers`; these sit under the whole-robot layers). The
   gate under the layers checks the joint commands they send; with layers, a
   second gate on top checks what the caller sends.

`overrides` maps a part path to overrides merged over the config, so
host-specific values (network interface, IP, `read_only`) can come from the
command line: `{"body": {"driver": {"network_interface": "eth0"}}}` for a single
robot, `{"left/body": {"driver": {"ip": ...}}}` for a part of an assembly.

`sim=True` (or a `sim:` section with `sim` left `None`) builds the robot in one
shared MuJoCo world instead (`hirobot.sim.sim_world`): each body from its device
file's `model:`, at its placement, a view of the world as its driver, mounted
cameras rendered by the world, no real driver built. `sim=False` builds the real
robot of a config that has a `sim:` section. `sim` may also be a mapping merged
over the config's `sim:` (`{"realtime": False, "viewer": True}`).

A simulated camera delivers at its frame rate, like a real one: it renders a new
frame when simulated time reaches its next frame time, every `1 / fps` (its
`Camera.fps`; without one, `sim.camera_fps`, default 30). In between an
observation carries the same image object and the same stamp (`t`, `seq`), so
treat images as read-only. `camera_fps: null` renders a camera without `fps` on
every observation. The first observation after `connect`, `reset` or `set_state`
always renders. A `mujoco` body on its own takes the same `camera_fps` driver
option.

To rehearse a motion in simulation before running it on hardware you can also
open the real robot with `read_only: true`, build its `*_mujoco` config as a
second robot, and start that one from the real pose with `MujocoRobot.set_state`
(see `examples/rehearse_in_sim.py`).

Raises `ValueError` for a camera / end-effector / audio / layer spec
without `type`, an end-effector driver that does not support the interface its config asks for, a `layers:` that is not a list, or a speaker driver listed as a
microphone (or the reverse). Raises `KeyError` for an unknown driver or layer name.

### Layers

A layer is a `Robot` that wraps another robot and changes how it is commanded:
a whole-body controller that takes `BASE_VELOCITY` / `MOTION_CLIP` / `CARTESIAN`
and sends balanced joint targets to the robot underneath every tick. hiRobot
ships one, the IK layer [`mink_ik`](#mink_ik-built-in-ik-layer); plugin packages
(hiWBC) register the others.

```python
from hirobot import Layer, Robot, register_layer

@register_layer("my_wbc")
class MyWbc(Layer):
    def __init__(self, inner: Robot, **config):
        super().__init__(inner)
    def _send_action(self, action): ...   # compute, then self.inner.send_action(...)
```

`register_layer`, `Layer` and the methods it implements are the extension side, see
[Writing a layer](#writing-a-layer); a user only writes the `layers:` entry.

```yaml
layers:                     # first entry innermost
  - type: my_wbc            # registered layer name
    controller: sonic       # every other key -> the layer's constructor
```

`Layer` forwards everything to `inner` by default: `embodiment`, the
capability declarations (`action_modes`, `motion_spaces`, `cartesian_targets`,
`direct_joints`), the devices (`cameras`, `microphones`, `speakers`), the
lifecycle, observations, `get_status`, `hold`, gains and onboard skills; `fault`
reports the layer's own fault or the one below. A subclass overrides what it
changes. `estop()` reaches the robot at the bottom first, then each layer stops
its own work. How to write one: [Writing a layer](#writing-a-layer).

A layer sends its joint commands through `inner.send_action`, so the gate
`make_robot` attaches underneath checks them. The layer's `inner` is the whole
robot: with real dexterous hands it can drive the fingers itself, or leave them
free (`direct_joints`) for the caller to command by `Action.joint_names`; the
cameras and audio devices are reachable through it as on the robot below. Register
layers from the same `hirobot.drivers` entry point as drivers.

**Layer skills.** A layer can run its own skills over many control cycles (a
behavior, a macro), apart from the vendor's onboard skills:
`list_layer_skills()` (this layer's and the layers' below), `run_layer_skill(name, args=None) -> RobotSkillResult`
(arguments checked against the schema, returns once started),
`stop_layer_skill()` and `layer_skill_running` (true while a skill of this layer
or one below runs). The four are on every `Robot`, so callers need not check
whether the outermost robot is a layer. While a skill runs, the layer
holds the stack's [command lease](#command-arbiter) with `Priority.LAYER_SKILL`:
a caller's lease is preempted, and direct commands raise `RobotBusyError` until
the skill ends or `stop_layer_skill()` is called. A `TAKEOVER` preempts the
skill (it is stopped). `estop()` ends it, so the caller can `reset()`.

#### `mink_ik` (built-in IK layer)

`mink_ik` (`hirobot.layers.MinkIkLayer`) takes end-effector targets and sends
joint targets: each `CARTESIAN` or `CARTESIAN_DELTA` action runs a few steps of
differential IK ([mink](https://github.com/kevinzakka/mink)) on a MuJoCo model of
the robot and sends one `POSITION` action with every joint through
`inner.send_action`, so the gate below checks it. Every other mode passes
through unchanged. Install with `pip install 'hirobot[ik]'` (mink needs Python
3.11+); without it `hirobot` still imports and the layer stays registered, and
building it raises an `ImportError` that names the extra.

```yaml
layers:
  - type: mink_ik
    model: unitree_g1           # MJCF path, or a config whose body has model: mjcf:
    frames:                     # optional; default: every end-effector with a frame
      right_hand: {name: right_wrist_yaw_link, type: body}   # body | site | geom
```

| Key | Default | Meaning |
|---|---|---|
| `model` | the simulation's model | MJCF path (`~` expanded), or the name / path of a config whose one body has `model: mjcf:`. Without it the layer uses the MuJoCo model of a simulated robot below, on `connect()`; a real robot needs it |
| `frames` | every end-effector with a `frame` | End-effector name -> `{name, type}` of an MJCF body, site or geom |
| `dt` | one control period | Seconds one `send_action` integrates |
| `iterations` | `3` | QP steps per `send_action` |
| `position_cost`, `orientation_cost` | `1.0`, `0.5` | Weights of the end-effector tasks |
| `posture_cost` | `0.01` | Pulls every joint towards the pose at the last re-seed (keeps legs and waist still) |
| `damping`, `lm_damping` | `0.001`, `0.001` | QP damping, Levenberg-Marquardt damping of the end-effector tasks |
| `max_joint_velocity` | `3.0` | rad/s (or m/s) per joint; `null` turns the velocity limit off |
| `joint_limits` | `true` | Keep the solution inside the model's joint ranges |
| `solver` | `daqp` | QP solver name (qpsolvers) |

- **Frame.** Targets and the `ee_poses` the layer reports are in the robot's
  base frame: the root body of the tree the joints are on (a humanoid's pelvis,
  an arm's base link), as the MuJoCo driver reports `ee_poses`. A floating
  base's free joint is frozen in the QP, never solved for.
- **Targets.** `CARTESIAN`: `ee_targets[name]` is `[x, y, z, qx, qy, qz, qw]`.
  `CARTESIAN_DELTA`: `[dx, dy, dz, dax, day, daz]` per tick (position plus an
  axis-angle rotation in the base frame), added to that end-effector's last
  target, not to the measured pose. An end-effector the action leaves out keeps
  its last target. Targets are re-seeded from the measured pose on first use,
  after `reset()`, after `hold()` and after a passed-through command. An unknown
  end-effector or a wrong-length target raises `SafetyError`.
- **Other joints.** Joints of the robot that the IK model does not have (fingers)
  are in `direct_joints`; set them with `Action.joint_names` / `joint_target`.
  `gripper` is passed on.
- **Failures.** A solver error or a non-finite result sends the previous joint
  target again and logs a warning; it never raises. An unreachable target is
  not an error: the IK moves as close as the limits allow.
- **Introspection.** `layer.commanded_ee_poses` is the forward kinematics of the
  last joint target (the difference to `ee_poses` is tracking error, e.g. an arm
  sagging on PD); `layer.stats` has `count`, `mean_ms`, `max_ms`, `failures`.
- **Joint targets.** `layer.commanded_joint_targets` is joint name -> the
  `POSITION` target last sent down for an IK command, every joint of that action
  (a recorder reads it after each command to store the IK's joint targets next to
  the requested `CARTESIAN` action). After a solver failure it keeps the resent
  previous target. It is `None` before the first IK command and after `reset()`,
  `hold()` or a passed-through command. It returns a copy.

### Registry

```python
from hirobot import Robot, register_body, get_body, available_bodies

@register_body("my_robot")
class MyRobot(Robot):
    ...
```

| Function | Description |
|---|---|
| `register_body(name)` | Class decorator. Raises `ValueError` if the name is taken or the class is not a `Robot` |
| `get_body(name) -> Type[Robot]` | Raises `KeyError` listing the available names |
| `available_bodies() -> List[str]` | Sorted registered names |

Built-in drivers register when `hirobot.drivers` is imported (`make_robot` does
this). Installed packages can add drivers through the `hirobot.drivers` entry-point
group; they load with the built-ins. Vendor SDKs are imported lazily on `connect()`,
which first calls `require_sdk` (below).

| Name | Class | Module | Target |
|---|---|---|---|
| `toy` | `ToyRobot` | `drivers/sim/toy/` | Toy kinematic integrator, no dependencies. A floating-base toy reports `base_linear_acceleration` as an IMU does, `[0, 0, 9.81]` at rest |
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo physics, a robot of its own; renders cameras that have a `mount`, at their `fps` (`camera_fps`, default 30, for a camera without one; `None`: every observation); `ee_poses` for end-effectors that have a `frame`; `set_state` starts it from a measured pose. With a floating base, `base_linear_acceleration` is what an IMU accelerometer reads, as on the real Unitree G1: specific force (acceleration minus gravity) in the base frame, m/s², about `[0, 0, 9.81]` upright at rest; a welded base reads `[0, 0, 9.81]` |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1 (精灵 G01): arms, waist, head and grippers over the GDK's DDS topics; chassis over its controller box |
| `ros2` | `Ros2Robot` | `drivers/body/generic/ros2/` | Any ROS 2 robot, set up by config: `JointState` / `Imu` / `Odometry` in; `JointTrajectory`, `Float64MultiArray` or a custom message (field map) out. AgiBot X1 is one config of it ([guide](setup/ros2.md)) |
| `franka_droid` | `FrankaDroidRobot` | `drivers/body/franka/droid/` | Franka FR3 and its gripper behind a DROID robot server (zerorpc) |
| `ur_rtde` | `URRobot` | `drivers/body/ur/rtde/` | Universal Robots UR5e |
| `agilex_piper_sdk` | `PiperRobot` | `drivers/body/agilex/piper_sdk/` | AgileX PiPER and its gripper over SocketCAN (`piper_sdk`) |

Constructor arguments of each hardware driver are documented in its
[setup guide](setup/README.md).

### SDK manifests (`hirobot.loading.sdk`)

Every hardware driver directory has a `manifest.yaml` naming the vendor SDK it
needs, the oldest version that works, the install command, and which model ran
on which SDK version on real hardware:

```yaml
sdk:
  - distribution: unitree_sdk2py    # importlib.metadata name
    module: unitree_sdk2py          # import name (default: distribution)
    minimum: "1.0.1"
    install: pip install -e <clone of https://github.com/unitreerobotics/unitree_sdk2_python>
verified:                           # append-only, one entry per model x SDK version
  - model: Unitree G1 (29 DoF)
    sdk: {unitree_sdk2py: "1.0.1 (a035ade)"}
    date: 2026-11-04
maintainers: []
```

`require_sdk(package, who=None)` reads it: a missing SDK raises `ImportError` with the
install command, one older than `minimum` raises, and one newer than every version in
`verified` logs a warning once and runs. It returns `{distribution: installed version}`.
`load_manifest(package)` returns the parsed `Manifest`.

## Robot

`hirobot.core.robot.Robot`, the abstract driver contract every robot, simulator and
wrapper implements.

**Threads.** `get_observation`, `get_status`, `now` and `estop` may be called
from any thread at any time. Every other method (`send_action`, `hold`,
`reset`, `run_robot_skill`, `run_onboard_skill`, `set_gains`, `connect`, `disconnect`, ...) is
called by one holder at a time: the holder of the [command
lease](#command-arbiter), or the single caller when nobody holds one. A lease
may be passed between threads, but its calls must not overlap. Drivers do not
promise that two threads sending at once get any particular order.

| Member | Kind | Description |
|---|---|---|
| `embodiment` | abstract property | The `Embodiment` this driver exposes |
| `connect()` | abstract | Open the connection. Called once before control |
| `reset(seed=None)` | abstract | Bring the robot to a safe initial state. Simulators use `seed` to pick the scene (e.g. an env seed or init-state index); other drivers ignore it |
| `get_observation() -> Observation` | abstract | Read sensors |
| `get_status() -> RobotStatus` | optional | Health and command progress, see [`RobotStatus`](#robotstatus). By default only `timestamp` and `fault` |
| `send_action(action)` | concrete | `check_action`, then the driver's `_send_action` ([SPI](#writing-a-driver-or-a-layer-spi)). Raises `SafetyError` on refusal and nothing is sent, including when `action.joint_names` lists a joint outside `direct_joints`. Raises `RobotFaultError` in the fault state, and `RobotBusyError` (a `RuntimeError`) when someone else holds the robot's [command lease](#command-arbiter), e.g. it is a part of an assembly and the call does not come through the assembly |
| `check_action(action) -> Action` | concrete | Every check `send_action` runs, without sending; returns the (possibly clamped) action. An assembly checks every part's piece before sending to any |
| `fault` | property | Why the robot refuses commands, or `None`. Set when a driver or layer damped or stopped itself, and by `estop()`. Cleared when `reset()` returns, unless `estop()` ran meanwhile. A wrapper (`RobotAssembly`) reports its inner robot's too. Also `get_status().fault` |
| `estop()` | concrete | Stop now; safe from any thread at any time, skips the safety gate and the [command arbiter](#command-arbiter). A long call on another thread (a `reset` ramp, a wait in `connect`) raises `RobotStoppedError` at its next step; the robot enters the fault state; a command already being sent finishes (at most `ESTOP_LOCK_TIMEOUT_S`, 0.2 s) and none goes out after; then the driver's own stop runs. Unitree and MuJoCo damp (Unitree with `builtin_skills` stops the skill), AgiBot G1 sends `MotionStop`, UR decelerates (`servoStop` / `speedStop` / `stopJ`), Franka stops its position controller; drivers without `damp()` (`toy`, `fourier_grx`, `ros2`) only refuse further commands. `RobotAssembly` e-stops its body and every end-effector, or every part. On a layer, the robot at the bottom of the stack is e-stopped first, then each layer stops its own work, outermost first (see [Layer hooks](#layer-hooks); [Layers](#layers)) |
| `claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None, parts=None) -> CommandLease` | concrete | Take the robot's command stream. See [Command arbiter](#command-arbiter) |
| `command_events() -> List[CommandEvent]` | concrete | The arbiter's audit events, oldest first; empty without an arbiter. See [Audit](#audit) |
| `add_listener(on_observation=None, on_command=None) -> ListenerHandle` | concrete | Callbacks for every top-level `get_observation()` and `send_action` / `hold`. See [Listeners](#listeners) |
| `command_timeout_s` | attribute | Seconds without a new `VELOCITY` / `BASE_VELOCITY` command before the driver stops the motion and logs a warning (`hirobot.control.watchdog.CommandWatchdog`); not a fault, the next command runs as usual. Default 0.2 s on `toy`, `mujoco` (simulated time), `unitree_sdk2` (zero velocity, kp 0) and `ur_rtde` (`speedStop`); `agibot_gdk` has its own `command_timeout` (0.3 s) for the chassis; `None` on the other drivers. Set `body.command_timeout_s: null` to turn it off. Position targets are not watched |
| `action_modes` | class attribute | Modes the robot accepts. Callers that build actions read it |
| `motion_spaces` | class attribute | `MotionClip.space` values the robot tracks under `MOTION_CLIP`. Empty by default |
| `direct_joints` | class attribute | Joints a caller may command by name beside a non-joint mode. Empty by default |
| `cartesian_targets` | property | End-effectors that take `CARTESIAN` pose targets: every end-effector when `CARTESIAN` is in `action_modes`, else none. Overridden by robots that track only some |
| `cameras`, `microphones`, `speakers` | attributes | Attached devices by name: camera drivers, microphones and speakers. Empty on a bare driver; the assembly `make_robot` builds fills them, and every layer passes them up. See [Extension points](#extension-points-cameras-end-effectors-and-audio) |
| `camera_info(name)` | method -> `CameraInfo` | Intrinsics and mount (extrinsics) of camera `name`, to turn `images[name]` and `depth[name]` (metres, registered to the colour image) into 3D. Intrinsics come from the first source that has them: the config's `intrinsics:`, the camera driver (after `connect()`; RealSense, Orbbec, ZED read them from the device), else an ideal pinhole from the mount's `fovy`. They are rescaled to the camera's declared `width` / `height`. `mount` is the pose of the optical frame in a robot link; for a head or wrist camera, composing it with the link's current pose is the caller's step. A child part's camera is named `<part>/<name>`. `KeyError` for an unknown camera |
| `accepts_parts` | class attribute | Whether `send_action` takes `Action.parts`: `True` on an assembly with child parts, else `False` |
| `now()` | optional | The robot's clock in seconds, what `Observation.stamps` times are on: `time.monotonic()` by default, simulated time in `toy` / `mujoco`. Wrappers return the inner robot's |
| `hold()` | optional | Keep the robot where it is, e.g. after a policy is stopped. By default sends the current `joint_pos` as a POSITION action when the driver accepts POSITION, otherwise does nothing. A simulator that takes only Cartesian deltas does nothing; Unitree with `builtin_skills` does nothing (the motion service keeps the robot standing). A driver with its own balance loop (a whole-body-controlled humanoid) must override it to command standing still (base velocity `[0, 0, 0]`): sending `joint_pos` freezes the joints and the humanoid falls. Goes through the safety gate |
| `set_gains(kp, kd)` | optional | Per-joint PD gains. No-op by default |
| `has_onboard_skills` | property | Whether the driver offers onboard skills at all. `False` by default |
| `list_onboard_skills() -> List[RobotSkillSpec]` | optional | The skills the robot's own controller runs. Empty by default. See [Onboard skills](#onboard-skills) |
| `run_onboard_skill(name, args=None) -> RobotSkillResult` | concrete | Check `args` against the skill's schema, then run it. Raises `RobotSkillError` |
| `stop_onboard_skill()` | optional | Stop the running skill. No-op by default |
| `list_layer_skills() -> List[RobotSkillSpec]` | concrete | The skills the stack's layers run. Every robot has it; without layers it is empty. See [Layer skills](#layers) |
| `run_layer_skill(name, args=None) -> RobotSkillResult` | concrete | Start a layer skill. Without layers it raises `RobotSkillError` |
| `stop_layer_skill()` | concrete | Stop the running layer skill. No-op without layers |
| `layer_skill_running` | property | Whether a layer skill (of any layer in the stack) holds the command stream. `False` without layers |
| `list_robot_skills() -> List[RobotSkillSpec]` | concrete | Every skill the robot can run, layer skills and onboard skills in one list. See [Skills](#skills) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | Start a skill from `list_robot_skills()`, whichever kind it is |
| `stop_robot_skill()` | concrete | Stop the running skill, of either kind |
| `robot_skill_running` | property | Whether a layer skill is running. Onboard skills are not tracked |
| `disconnect()` | optional | Release and stop. No-op by default |
| `safety` | attribute | The attached gate, or `None` |
| `simulated` | class attribute | `True` for simulator drivers. With a simulated driver `make_robot` builds no end-effector drivers and only the cameras and audio devices whose class is also `simulated` (video file, WAV, the host's microphone) |
| `success`, `done`, `reward` | attributes | Episode outcome. Every driver in hiRobot (`toy`, `mujoco`, the real robots) leaves them `None`: they run no task. Task environments set them, e.g. hiBench's benchmark environments and its referee. A control loop can stop on them |

## Embodiment

`hirobot.core.embodiment`. Pydantic models describing what a robot looks like to a
policy, independent of the driver. Built from a config (a device file or an
`assembly:` tree) with `load_embodiment(config)`; for an assembly it is the
merged tree, each part's names under its `<part>/` prefix, and `parts` maps each
path to its joint span and children.

### `Embodiment`

| Field | Type | Default | Meaning |
|---|---|---|---|
| `name` | `str` | required | Identifier |
| `kind` | `EmbodimentKind` | `HUMANOID` | `"humanoid"` or `"arm"` |
| `control_hz` | `int` | `30` | Rate a control loop paces real robots to |
| `joints` | `List[Joint]` | `[]` | Actuated joints, in the canonical vector order |
| `floating_base` | `bool` | `True` | Robot has a moving base (IMU / base state) |
| `end_effectors` | `List[EndEffector]` | `[]` | Task-space frames, hands and grippers |
| `cameras` | `List[Camera]` | `[]` | Camera streams |
| `microphones` | `List[AudioDevice]` | `[]` | Microphones |
| `speakers` | `List[AudioDevice]` | `[]` | Speakers |
| `battery_soc_limit` | `(warn, shutdown)` or `None` | `None` | Battery charge (percent) where a monitor flags it as low (`warn`) and where it is unsafe to keep commanding (`shutdown`). Both within 0-100, `warn >= shutdown`. The gate uses `shutdown` when `safety:` sets no `min_battery_soc` |
| `model` | `ModelRef` or `None` | `None` | Model file of the robot's one body, from its device file's `model:`. `None` with more than one body or no `model:`. Filled by `load_embodiment` and `make_robot`, real robots included |

Properties: `joint_names -> List[str]`, `num_dofs -> int`.

Validation raises `ValueError` on duplicate joint names, an end-effector listing
an unknown joint, a joint claimed by two end-effectors, or an end-effector with a
`driver` but no `joints`.

### `Joint`, `EndEffector`, `Camera`, `AudioDevice`, `EmbodimentKind`

| Model | Field | Type | Meaning |
|---|---|---|---|
| `Joint` | `name` | `str` | Unique name; the key for DoF mapping |
| | `position_limit` | `(lower, upper)` or `None` | Position bounds (rad). End-effector drivers clip to it |
| | `torque_limit` | `float` or `None` | Max absolute torque (Nm) |
| | `axis` | `(x, y, z)` or `None` | Unit vector the joint turns about, in the frame of the link the joint moves (as URDF `<axis>` and MJCF `axis`), not the world frame |
| | `parent_link` | `str` or `None` | Link the joint hangs from (URDF `<parent>`) |
| | `temperature_limit` | `(warn, shutdown)` or `None` | Motor temperatures (degC): `warn` is where a monitor flags the joint as hot, `shutdown` where it is unsafe to keep commanding it. Both > 0, `warn <= shutdown`. The gate uses `shutdown` when `safety:` sets no `max_joint_temperature` |
| `EndEffector` | `name` | `str` | Key in `ee_poses` / `ee_targets` / `gripper` |
| | `has_gripper` | `bool` | Has a gripper commanded through `Action.gripper`. Required for an end-effector driver without `joints` (`opening` interface) and not allowed with `joints` |
| | `joints` | `List[str]` | Embodiment joints this end-effector actuates (finger joints), in its driver's motor order |
| | `driver` | `dict` or `None` | End-effector driver spec: `type` names a registered end-effector driver, other keys are its arguments. Needs `joints` or `has_gripper: true` |
| | `frame` | `EndEffectorFrame` or `None` | Link the pose is read from: `body` (link name), `pos` (m, in that link), `quat` `[x, y, z, w]` in that link. The `mujoco` body fills `ee_poses` for end-effectors with a frame; drivers whose SDK reports the pose (UR, Franka) ignore it; `Kinematics` computes the same pose from joint angles |
| `Camera` | `name` | `str` | Key in `Observation.images` |
| | `width`, `height` | `int` | Frame size as delivered |
| | `driver` | `dict` or `None` | Camera-driver spec: `type` names a registered camera driver, other keys are its arguments. Omit for cameras the body driver renders itself (simulators). The old key `source` is an error |
| | `depth` | `bool` | RGB-D camera: its driver also delivers a depth image registered to the colour image, in `Observation.depth` under the same name. Default `False` |
| | `fps` | `float` or `None` | Nominal frame rate. `stale_sources` uses it; `make_robot` warns when `control_hz` is more than twice it. A simulated camera renders at this rate (without it, at `sim.camera_fps`) |
| | `mount` | `CameraMount` or `None` | Where the camera sits: `body` (link name), `pos` (m, in that link), `quat` `[x, y, z, w]` of the optical frame (z forward, x right, y down) in that link, `fovy` (vertical field of view, degrees). These are the camera's extrinsics: the `mujoco` body renders mounted cameras into `images` at that pose; for a real camera they are the calibrated pose, read back with `Robot.camera_info`. A config gives it through the camera's `placement` |
| | `intrinsics` | `CameraIntrinsics` or `None` | Fixed intrinsics from a calibration (the camera file's `intrinsics:`). They win over what the driver reads from the device |
| `CameraIntrinsics` | `width`, `height` | `int` | Image size these intrinsics are for. In a config they default to the camera's `width` / `height` |
| | `fx`, `fy`, `cx`, `cy` | `float` | Pinhole focal lengths and principal point, pixels. Pixel `(u, v)` has its origin at the top-left corner, `u` right, `v` down, pixel centres at integers. A point `(x, y, z)` in the optical frame (OpenCV: z forward, x right, y down; m) projects to `u = fx·x/z + cx`, `v = fy·y/z + cy` before distortion |
| | `model`, `coeffs` | `str`, `tuple` | Distortion: `none` (default, no coefficients), `brown_conrady` / `plumb_bob` (OpenCV `k1, k2, p1, p2, k3`), `inverse_brown_conrady` (RealSense, same 5), `kannala_brandt` (OpenCV fisheye `k1..k4`). All-zero coefficients become `none` |
| | `K`, `scaled(w, h)`, `from_fovy(w, h, fovy)` | | `K` is the 3x3 numpy camera matrix. `scaled` gives the same camera at another resolution (a resize, no crop). `from_fovy` is an ideal pinhole with the principal point at the image centre, what a simulator renders |
| `CameraInfo` | `name`, `intrinsics`, `intrinsics_source`, `mount`, `depth` | | What `Robot.camera_info` returns. `intrinsics_source` is `config`, `driver`, `fovy` or `None` |
| `AudioDevice` | `name` | `str` | Key in `robot.microphones` / `robot.speakers` |
| | `driver` | `dict` or `None` | Audio-driver spec: `type` names a registered microphone or speaker driver, other keys are its arguments |
| `ModelRef` | `kind` | `"mjcf"` or `"urdf"` | Format of `path`. With both in `model:`, `mjcf`. Frozen and hashable; fields added later are optional |
| | `path` | `str` | Absolute path, `~` expanded. Not checked to exist |
| | `package_roots` | `Dict[str, str]` | URDF only: package name -> absolute directory for `package://` mesh paths. Empty for MJCF |

`EmbodimentKind` is a string enum with `HUMANOID = "humanoid"` and `ARM = "arm"`.

## Observation, Action, ActionChunk, ControlMode

`hirobot.core.types`. Pydantic models that accept numpy arrays. Every optional field
defaults to `None` or `{}`, so a driver fills only what its robot provides.

### `ControlMode`

String enum selecting how a driver interprets an `Action`.

| Value | Meaning | Fields used |
|---|---|---|
| `POSITION` (`"position"`) | Joint position targets | `joint_target` |
| `VELOCITY` (`"velocity"`) | Joint velocity targets | `joint_target` |
| `TORQUE` (`"torque"`) | Joint torque targets | `joint_target` |
| `PD` (`"pd"`) | Position targets tracked with per-command gains | `joint_target`, `kp`, `kd` |
| `CARTESIAN` (`"cartesian"`) | End-effector pose targets (7-vector) | `ee_targets` |
| `CARTESIAN_DELTA` (`"cartesian_delta"`) | Per-tick end-effector deltas `[dx, dy, dz, dax, day, daz]` (axis-angle), as robosuite OSC_POSE takes them | `ee_targets` |
| `BASE_VELOCITY` (`"base_velocity"`) | Walk at `[vx, vy, wz]` in the body frame (m/s, m/s, rad/s) | `base_velocity` |
| `BASE_POSE` (`"base_pose"`) | Walk to `[x, y, yaw]` in the frame of `Observation.base_position`, then stand still | `base_target` |
| `MOTION_LATENT` (`"motion_latent"`) | An opaque motion token (e.g. a SONIC latent) | `latent` |
| `MOTION_CLIP` (`"motion_clip"`) | Track a reference motion | `motion`, `append` |

The last four command a robot that moves itself (a whole-body controller, see
[Layers](#layers)); `CARTESIAN` may also carry `base_velocity` there (reach while
walking). `JOINT_MODES` is the tuple of the first four joint modes.

Not every driver supports every mode: read `Robot.action_modes`, and see the
driver's setup guide.

### `Observation`

| Field | Type | Filled by |
|---|---|---|
| `joint_pos` | `(N,)` | every driver |
| `joint_vel` | `(N,)` | every driver |
| `joint_torque` | `(N,)` | drivers that measure or estimate torque |
| `base_orientation` | `(4,)` quaternion | floating-base robots |
| `base_angular_velocity` | `(3,)` | floating-base robots (IMU) |
| `base_linear_acceleration` | `(3,)` | floating-base robots (IMU) |
| `base_position` | `(3,)` | usually simulators only |
| `base_linear_velocity` | `(3,)` | usually simulators only |
| `ee_poses` | `Dict[str, (7,)]` | arms / task-space drivers; `[x, y, z, qx, qy, qz, qw]` in the robot's base frame. Drivers convert their SDK's form (Franka's 4x4 `O_T_EE`, UR's axis-angle) |
| `gripper` | `Dict[str, float]` | robots with grippers: measured opening per end-effector, in [0, 1], 0 closed and 1 open (the scale of `Action.gripper`). Filled by the body driver, or by a gripper's own end-effector driver (`opening` interface) |
| `images` | `Dict[str, HxWxC uint8]` | cameras, keyed by `Camera.name` |
| `depth` | `Dict[str, HxW float32]` | cameras with `depth: true`: depth in metres registered to `images[name]` (same size, same pixels), 0 where there is no reading |
| `stamps` | `Dict[str, Stamp]` | every driver sets `stamps["body"]`; hands, grippers with their own topic and cameras add theirs under their name (`left/body` etc. for a part of an assembly). `Stamp(t, seq)`: `t` is when the reading arrived on `robot.now()`, `seq` counts the source's readings from 1. Age of a source: `robot.now() - obs.stamps[name].t`; a camera frame seen again has the same `seq`. `hirobot.core.stamps.stale_sources(obs, robot.now(), max_age=None, embodiment=None)` lists sources older than `max_age` (seconds, or per source) or, by default, 3 nominal periods (`Camera.fps`, `control_hz`) |
| `instruction` | `str` or `None` | task instruction for a VLA |
| `extra` | `Dict[str, Any]` | driver-specific signals (tactile, raw state, ...) |

### `Action`

| Field | Type | Default | Meaning |
|---|---|---|---|
| `mode` | `ControlMode` | `POSITION` | How to interpret the fields |
| `joint_target` | `(N,)` or `None` | `None` | Per-joint target in the unit of `mode` |
| `kp`, `kd` | `(N,)` or `None` | `None` | Per-joint gains (`PD`) |
| `joint_names` | `List[str]` or `None` | `None` | Non-joint modes only: the joints `joint_target` / `kp` / `kd` cover, one value per name. Must be in `Robot.direct_joints` |
| `ee_targets` | `Dict[str, (7,) or (6,)]` | `{}` | Pose targets (`CARTESIAN`) or deltas (`CARTESIAN_DELTA`) |
| `gripper` | `Dict[str, float]` | `{}` | Gripper opening target per end-effector, in [0, 1], 0 closed and 1 open, sent in any mode. Drivers convert to their hardware's range. A gripper with its own end-effector driver refuses values outside [0, 1] (`ValueError`) |
| `base_velocity` | `(3,)` or `None` | `None` | `[vx, vy, wz]` (`BASE_VELOCITY`, or with `CARTESIAN`) |
| `base_target` | `(3,)` or `None` | `None` | `[x, y, yaw]` (`BASE_POSE`) |
| `latent` | `(D,)` or `None` | `None` | Motion token (`MOTION_LATENT`) |
| `motion` | `MotionClip` or `None` | `None` | Reference motion (`MOTION_CLIP`) |
| `append` | `bool` | `False` | `MOTION_CLIP`: queue after the playing clip instead of replacing it (streamed generation) |
| `extra` | `Dict[str, Any]` | `{}` | Free-form outputs |
| `parts` | `Dict[str, Action]` | `{}` | For a robot with parts (an `assembly:` config): part name -> that part's action, in its own names and mode. A part's action may carry `parts` of its own for a nested node. Every other field but `extra` must then be empty; a robot without parts refuses it |

Construction raises `ValueError` for `MOTION_CLIP` without `motion`, `BASE_POSE`
without `base_target`, `joint_names` with a joint mode, duplicates or no
`joint_target`, or `parts` beside another command field. (The old name
`members` raises a `ValueError` pointing to `parts`.)

`joint_names` lets a caller command joints that whatever runs the robot leaves
free, in the same action: the fingers of a humanoid whose whole-body controller
drives only the body, say. A controller that drives the hands too lists no
finger joints in `direct_joints`, and such an action is refused.

### `MotionClip`

A reference motion for a robot to track, independent of any controller. Frames
are evenly spaced at `fps`; every array set has the same number of frames `T`.

| Field | Type | Meaning |
|---|---|---|
| `fps` | `float` | Frame rate, positive |
| `space` | `str` | `"joint"` (the robot's joints, by name) or `"smpl"` (human motion); see `MOTION_SPACES` |
| `joint_names` | `List[str]` | `joint`: column names of `joint_pos` / `joint_vel`; may be a subset of the robot's joints |
| `joint_pos` | `(T, n)` | `joint`: joint positions (rad) |
| `joint_vel` | `(T, n)` or `None` | `joint`: joint velocities (rad/s), optional |
| `smpl_joints` | `(T, J, 3)` | `smpl`: SMPL joint positions (m), root frame |
| `root_pos` | `(T, 3)` or `None` | Root position (m) |
| `root_quat` | `(T, 4)` or `None` | Root orientation `[x, y, z, w]` |
| `extra` | `Dict[str, Any]` | Free-form |

`num_frames` and `duration_s` (`num_frames / fps`) are properties;
`arrays()` returns the per-frame arrays that are set. Construction raises
`ValueError` on a wrong shape, a frame-count mismatch, or missing fields for the
`space`. A clip is already a sequence, so it travels as one
`Action(mode=MOTION_CLIP)`, not in an `ActionChunk`.

### `RobotStatus`

What `robot.get_status()` returns: health and housekeeping for the operating
layer (an agent, a UI). Kept out of `Observation`, which a remote policy
forwards to the model every tick. Fields a robot does not report stay empty.

| Field | Type | Meaning |
|---|---|---|
| `timestamp` | `float` or `None` | `time.time()` when assembled |
| `battery` | `BatteryState` or `None` | `soc` (%), `voltage`, `current`, `temperature`, `health` (%), `cycles` |
| `joint_temperature` | `(N,)` or `None` | Per-joint motor temperature (degC) |
| `joint_faults` | `Dict[str, int]` | Joint name -> nonzero vendor fault code |
| `board_temperature` | `Dict[str, float]` | Board name -> degC |
| `fault` | `str` or `None` | As `robot.fault` |
| `command` | `CommandProgress` or `None` | Progress of the last high-level command; `None` when the robot reports none |
| `extra` | `Dict[str, Any]` | Driver- or layer-specific readings (e.g. `extra["wbc"]`) |
| `safety` | `Dict[str, Any]` | Gate counters: `audit`, `sim_check` |

`CommandProgress` is for commands a controller follows over many ticks, so a
caller can tell whether the robot arrived or the clip ended without knowing the
driver. The controller fills it (a whole-body-control layer; hiRobot's own
drivers leave it `None`) and keeps the last one after it ends, `done=True`,
until the next command replaces it. Layers pass it up unchanged.

| Field | Type | Meaning |
|---|---|---|
| `mode` | `ControlMode` | The command's mode |
| `done` | `bool` or `None` | `True` once a `BASE_POSE` goal is reached or a `MOTION_CLIP` has played out; `None` for a mode with no end (`BASE_VELOCITY`) |
| `goal` | `[x, y, yaw]` or `None` | `BASE_POSE` target, in the frame of `Observation.base_position` |
| `clip_time`, `clip_duration` | `float` or `None` | `MOTION_CLIP`: seconds played, and its length |

### `ActionChunk`

A horizon of `T` actions predicted in one inference call.

| Field | Type | Meaning |
|---|---|---|
| `mode` | `ControlMode` | Applies to every step |
| `joint_targets` | `(T, N)` or `None` | Per-step joint targets |
| `kp`, `kd` | `(N,)` or `None` | Gains shared across the chunk |
| `joint_names` | `List[str]` or `None` | Non-joint modes: the joints `joint_targets` covers (see `Action`) |
| `ee_targets` | `Dict[str, (T, 7) or (T, 6)]` | Per-step task-space targets |
| `gripper` | `Dict[str, (T,)]` | Per-step gripper commands |
| `base_velocity` | `(T, 3)` or `None` | Per-step base velocity |
| `base_target` | `(T, 3)` or `None` | Per-step base target |
| `latent` | `(T, D)` or `None` | Per-step motion tokens |
| `extra` | `Dict[str, Any]` | Free-form outputs |
| `parts` | `Dict[str, ActionChunk]` | A robot with parts: part name -> its chunk. All have the same horizon (`ValueError` otherwise), the other fields stay empty, and `step(i)` gives an `Action` with `parts` |

- `horizon -> int` (also `len(chunk)`): `T`, taken from `joint_targets`,
  `base_velocity`, `base_target` or `latent`, then `ee_targets`, then `gripper`;
  `0` if all are empty.
- `step(i) -> Action`: the `i`-th step as a single `Action`. Raises `IndexError`
  outside `[0, horizon)`.

## Skills

What a robot can do by name, for a caller (an agent, a voice front end) that
shows it to a model. Two kinds sit underneath, [onboard skills](#onboard-skills)
(the vendor's controller runs them) and [layer skills](#layers) (a layer runs
them); these four calls list and run both without the caller telling them apart.

```python
robot = make_robot("my_g1.yaml")
robot.connect()
for s in robot.list_robot_skills():          # RobotSkillSpec: name, description, parameters (JSON Schema), ...
    print(s.name, s.description)
robot.run_robot_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_robot_skill()
```

| Call | Description |
|---|---|
| `list_robot_skills() -> List[RobotSkillSpec]` | Layer skills first, then onboard skills. A layer skill with the same name as an onboard skill hides it (the outer one wins). In an assembly names carry the part path (`left/grasp`) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | Goes to `run_layer_skill` or `run_onboard_skill`, with their argument checks and lease rules: a layer skill holds the command stream until it ends. Raises `RobotSkillError` for an unknown name or bad arguments, `RobotBusyError` when someone else holds the robot |
| `stop_robot_skill()` | Stops both kinds: `stop_layer_skill()`, then `stop_onboard_skill()` |
| `robot_skill_running` | `layer_skill_running`. An onboard skill's `run_robot_skill` returns once the robot accepts it, and how long it runs is only estimated (`duration_s`) |

A `CommandLease` has `run_robot_skill` too. An onboard skill runs through the lease; a
layer skill preempts it, so claim again after the skill.

Drivers and layers do not implement these four. A driver offers onboard skills
through `list_onboard_skills` / `_run_onboard_skill`, a layer offers layer skills
through `_layer_skills` / `_start_layer_skill` ([SPI](#writing-a-driver-or-a-layer-spi)).

## Onboard skills

`hirobot.core.robot_skills`. An onboard skill is a motion the robot's own controller
carries out (walk, sit, wave), started by name instead of by streaming joint
targets. Each driver declares the skills hiRobot supports on its robot, so
callers (an agent, a voice front end) list them and show them to a model
instead of hard-coding any.

```python
robot = make_robot("my_g1.yaml")      # robot.builtin_skills: g1
robot.connect()
[s.name for s in robot.list_onboard_skills()]   # ['start_locomotion', 'walk', ...]
robot.run_onboard_skill("start_locomotion")
robot.run_onboard_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_onboard_skill()
```

### `RobotSkillSpec`

| Field | Type | Description |
|---|---|---|
| `name` | `str` | Passed to `run_onboard_skill` |
| `description` | `str` | One or two sentences for a model |
| `parameters` | `dict` | JSON Schema of the arguments (an `object` schema). Empty means none |
| `requires` | `List[str]` | Preconditions in plain words, e.g. `"robot standing on the ground"`. Informational |
| `duration_s` | `float` or `None` | Typical run time, if known |

### `run_onboard_skill(name, args=None) -> RobotSkillResult`

Checks `args` against `parameters` before anything reaches the robot, then
runs it on the driver. Supported schema keywords: property
`type` (number / integer / boolean / string), `minimum`, `maximum`,
`exclusiveMinimum`, `enum`, `default`, and `required`. Missing arguments take
their `default`; unknown arguments are refused. It returns once the robot has
accepted the command, not when the motion ends. `RobotSkillResult` carries
`name`, the checked `args` (defaults filled in) and `duration_s`: the driver's
estimate for these arguments if it gives one (G1 `walk`: its `seconds`; `turn`:
an upper estimate from the angle), else the spec's `duration_s`.

`RobotSkillError` is raised for an unknown skill, a bad argument, or a robot
that refused.

### Wrappers

`RobotAssembly` forwards all four skill calls to its body driver, and lists each
child part's skills under the part's prefix (`<part>/<skill>`).

### Drivers with skills

| Driver | Config | Skills |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 locomotion and arm gestures, see the [Unitree setup guide](setup/unitree.md#onboard-skills-g1) |

Every other driver offers none.

## Command arbiter

One caller at a time commands a robot. `make_robot` attaches a `CommandArbiter` (`hirobot.control.arbiter`) to the outermost robot of
the stack; a caller takes the command stream with `claim` and sends through the returned `CommandLease`:

```python
from hirobot import Priority, PreemptedError

with robot.claim("runner", on_preempt=lambda lease: stop_runner()) as r:
    r.send_action(action)       # also r.hold(), r.reset(), r.run_robot_skill(...)
```

| Item | Meaning |
|---|---|
| `robot.claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None) -> CommandLease` | Take the stream. A higher priority preempts a lower one at once; an equal or lower one raises `RobotBusyError` |
| `Priority` | `TAKEOVER` (a person: teleoperation, a joystick) > `LAYER_SKILL` (a layer while its skill runs) > `CALLER` (an application) |
| Direct calls while held | `send_action`, `hold`, `reset`, `run_onboard_skill` on the robot raise `RobotBusyError` (a `RuntimeError`). `estop`, `get_observation`, `get_status`, `now` are never checked |
| Nobody holding | Everyone may command, so single-caller scripts need no change. Config `arbiter: {require_claim: true}` refuses commands without a lease |
| Preemption | The old lease's `on_preempt(lease)` runs (errors are logged), then the robot `hold()`s, so it does not keep going toward the old target (a failing hold is logged). The old lease's next call raises `PreemptedError` (a `RobotBusyError`) |
| Release | `lease.release()` or leaving `with`. The stream is not given back to a preempted lease; it has to claim again |
| Threads | A lease may be used from any thread. A preemption waits for the old holder's call in flight up to `PREEMPT_LOCK_TIMEOUT_S` (0.2 s) before it holds |
| Layers | Commands a layer sends to `inner` are below the arbiter and not checked |

### Envelope

`claim(..., envelope={...})` gives one lease limits stricter than the robot's
gate, for an application that may do less than the robot can:

```python
lease = robot.claim("demo_app", envelope={"joint_limit": 1.0, "max_joint_step": 0.05,
                                          "allowed_modes": ["position"]})
lease.envelope    # the items in force: only those stricter than the robot's gate
```

| Key | Meaning |
|---|---|
| `joint_limit`, `max_joint_step`, `max_joint_error`, `gripper_range` | As in the `safety:` section (SafetyGate), for this lease's commands |
| `allowed_modes` | Control modes this lease may send, e.g. `["position", "pd"]` |

- Only tightens. An item not stricter than the robot's gate is ignored, with one
  warning for the claim; `gripper_range` is intersected with the robot's.
  An unknown key, a bad value, or `allowed_modes` the robot takes none of raise
  `ValueError` before anything is claimed.
- Checked in `lease.send_action`, before the robot's own gate. A refusal raises
  `SafetyError` starting with `envelope of lease '<name>':`. `clamp` follows the
  robot's gate.
- Fixed for the lease's life: release and claim again to change it.
- Applies to what the lease sends to the outermost robot only. It does not reach
  the gate under the layers, so a whole-body controller's joint commands keep
  the robot's own limits (an envelope there could keep the controller from
  balancing).

### Audit

The arbiter records events in a ring (`arbiter: {audit_size: 256}`) and counts
commands:

```python
robot.command_events()               # [CommandEvent, ...], oldest first
robot.get_status().safety["audit"]   # counts, the holder, the last refusal
```

| `CommandEvent.kind` | When |
|---|---|
| `claim`, `release` | A lease is taken or given back (a `TAKEOVER`'s pair marks an intervention) |
| `preempted` | A lease lost the stream; `detail` names who took it |
| `refused` | The gate or an envelope refused a call (`SafetyError`); `detail` is the reason |
| `busy` | A call or claim refused because another lease holds the robot, after a preemption or release, or under `require_claim` |

Each event has `t` (the robot's clock, `now()`), `wall` (`time.time()`), `lease`
(`None` for a direct call), `priority`, `call` and `detail`. Accepted commands
are counted, not recorded: `safety["audit"]` has `accepted`, `refused`,
`by_lease` (`"(direct)"` for calls with nobody holding; the 64 most recent
names), `holder` and `last_refusal`. A lease also has `accepted` and `refused`.

### Listeners

`robot.add_listener` lets a recorder or a monitor see what the robot reads and
is told, without being in the command path. Call it on the outermost robot
(what `make_robot` returns); on a bare driver it attaches an arbiter as `claim` does.

```python
import queue

q: "queue.Queue" = queue.Queue()
handle = robot.add_listener(on_observation=q.put, on_command=q.put)
...
handle.remove()
```

| Item | Meaning |
|---|---|
| `on_observation(obs)` | Every top-level `get_observation()` on this robot, with the returned `Observation`. Reads inside a command (`hold()` reading the pose) are not reported |
| `on_command(record)` | Every top-level `send_action` and `hold`, on the robot or a lease, accepted or not. `hold()`'s own `send_action` is not reported, nor is `reset`, a skill, or what a layer sends below |
| `CommandRecord` | `t` (`now()` when the call started), `call` (`send_action` / `hold`), `action` (as the caller passed it, before an envelope or the gate clamps it; `None` for `hold`), `lease` and `priority` (`None` for a direct call), `accepted`, `reason` (the message of the `SafetyError`, `RobotBusyError`, `RobotFaultError` or driver error), `parts` (the lease's part paths) |
| Threads | Callbacks run on the caller's thread after the call, so they must return quickly: put the item on a queue. Adding and removing listeners is safe from any thread |
| Errors | A callback that raises is logged (once per 10 s per listener; `handle.errors` counts them) and never changes the call's result |
| `ListenerHandle` | `remove()` (safe twice), `active` |

## SafetyGate

`hirobot.control.safety`. The check `Robot.send_action` runs on every action before it
reaches the driver.

```python
SafetyGate(num_dofs=None, joint_limit=None, clamp=False, gripper_range=None,
           max_joint_step=None, max_joint_error=None,
           min_battery_soc=None, max_joint_temperature=None, sim_check=None)
SafetyGate.from_embodiment(embodiment, **kwargs)   # num_dofs = embodiment.num_dofs
gate.check(action, embodiment=None) -> SafetyVerdict   # the checks on the action alone
```

The keys are the robot config's `safety:` section. The checks run as a chain, in
this order, and the first refusal stops it: the checks on the action alone (the
first four rows below), the checks that read the robot (the next four), then the
simulation check.

| Argument | Effect |
|---|---|
| `num_dofs` | `joint_target` must have this length (unless `joint_names` is set: then one value per name, and every name must be a joint of `embodiment`). `None` skips the check |
| `joint_limit` | Symmetric bound `[-joint_limit, joint_limit]` (rad) on `joint_target`. `None` skips it |
| `clamp` | `False` (default): out-of-bound targets are refused. `True`: they are clipped into bounds |
| `gripper_range` | `[low, high]` for gripper values; values are clipped into it. `None` skips it |
| `max_joint_step` | Largest change (rad) of a position target from the last target the robot accepted, per joint, per command. `None` skips it |
| `max_joint_error` | Largest distance (rad) of a position target from the measured position, per joint. `None` skips it |
| `min_battery_soc` | Refuse commands while `get_status().battery.soc` is below this (percent). `None` skips it |
| `max_joint_temperature` | Refuse commands while a joint in `get_status().joint_temperature` is hotter than this (degC). `None` skips it |

`from_embodiment` fills `min_battery_soc` and `max_joint_temperature` from the
embodiment when they are not given: `Embodiment.battery_soc_limit[1]` and the
smallest `Joint.temperature_limit[1]`. A value in `safety:` wins.

Always checked: `joint_target` is 1-D, finite, and gripper values are finite;
`base_velocity`, `base_target`, `latent`, every `ee_targets` entry and every
array of `motion` are finite.

Not checked: per-joint bounds (`Joint.position_limit`; only end-effector drivers clip to
them). On robots whose limits are not symmetric (AgiBot G1's elbows, UR, Franka)
the robot's own controller is the only per-joint limit. `joint_limit` applies to
`joint_target` in every joint mode, including `VELOCITY`, where it holds
velocities (rad/s).

### Step and error limits

`max_joint_step` catches a jump in the command stream (an operator's fast
move, a dropped tracking frame, a policy output that flips). `max_joint_error`
catches a target that drifts away from where the joint is: an arm blocked by a
table, pushed 0.05 rad further each command, passes every step check, and
lunges when the table moves. Under PD control a fast move keeps the target
ahead of the measured position, and a loaded arm sags below it, so set
`max_joint_error` wider than `max_joint_step`.

- Both apply to position targets: `POSITION` and `PD` actions, and the joints
  named in `joint_names` beside a non-joint mode (only those joints).
- `max_joint_step` is per command, not per second: a caller that pauses for
  10 s and then sends a large jump is refused, because the robot is still at the
  old target. A caller that sends at a low rate gets a tighter limit in rad/s.
- Where there is no last target the measured position is the reference: the
  first command after `connect`, `reset()` or `hold()`, after a velocity,
  torque, base or Cartesian command, and for joints the last command did not
  name. A refused command does not become the reference.
- `hold()` is not checked by these four checks or by the simulation check.
  A blocked arm's `hold()` targets the measured pose, which can be far from the
  last target, and refusing a stop would be worse.
- With `clamp: true` both clip the targets to the limit instead of refusing,
  like `joint_limit`.
- `max_joint_error` reads the observation on every position command;
  `max_joint_step` reads it only when it has no reference.

`min_battery_soc` and `max_joint_temperature` read `get_status()` at most once a
second and refuse every command except `hold()` while the reading is out of
range. A robot that reports no battery or temperatures is not refused. No
defaults: pick the values per robot.

`SafetyVerdict` has `allowed: bool`, `action` (possibly clamped, valid only when
allowed) and `reason: str`. `SafetyError` (a `RuntimeError`) is what
`send_action` raises with that reason.

### Simulation check

Optional, for a real robot whose bodies all have a `model:` (their MJCF). The
check world is built from the robot's own tree, like the `sim:` world: every
body placed as in the config, names prefixed with the part path, plus an
optional `scene`. For each command that passed every other check, the hardware
gate (under the layers) copies the robot's measured joint state into that world,
applies the command for `horizon_steps` control periods, and refuses it with
`SafetyError` when the prediction shows a joint torque above `torque_ratio`
times the joint's limit, a new or deeper contact (self-collision, two parts
touching, a part hitting something in the scene), or the robot sinking into the
ground. Contacts already there at the start are allowed to stay or get
shallower. People are not in the model. It only refuses: `clamp` does not
apply, and `hold()` is not predicted. It runs on the root's hardware gate only,
after the step and error limits.

A body without a `model:` is an error at `make_robot`: checking only some parts
would miss collisions. A part that moves by itself inside a tree (a mobile
base) is not supported yet. The old `model:` key (a `*_mujoco` config) is gone.

```yaml
safety:
  sim_check:
    scene: {mjcf: ./scenes/my_table.xml}   # optional: fixed obstacles
    fixed_base: [body]         # default: the robot's sim.fixed_base
    base_height: {body: 1.0}   # default: the robot's sim.base_height
    horizon_steps: 5           # control periods predicted per command
    torque_ratio: 1.0
    self_collision: true
    ground: true
    budget_s: null             # default: half a control period
    on_error: reject           # warn: for debugging only, never on the robot
    max_failures: 3
    ee_tolerance_m: 0.02       # measured vs model end-effector position, warned above
```

When the prediction itself fails (an exception, the model did not load, longer
than `budget_s`) the command is refused with "simulation check failed (not the
command)". After `max_failures` failures in a row the robot enters the fault
state; `reset()` leaves it. A command the check refuses is not a failure.
At every check the measured `ee_poses` are compared with the model's at the
same joint angles; one farther than `ee_tolerance_m` is warned about once and
counted (calibration or model off), and does not refuse the command.
`get_status().safety["sim_check"]` has the counts: `checked`, `rejected`,
`errors`, `skipped` (commands with no joint targets to predict, and holds),
`failures_in_row`, `p99_ms`, `last_reason`, `ee_mismatch` and `ee_error_m`
(the largest end-effector error at the last check, m). On the G1 model a 5-period
prediction takes about 1.5 ms on a desktop CPU; on the robot's PC2 it is not
measured yet. `safety: false` turns this check off together with the rest of the gate.

## RobotAssembly

A `Robot` wrapper that `make_robot` builds from the config. You rarely
construct it directly. Attributes it does not define (for example a driver's
`raw_state`) are forwarded to the wrapped driver.

### `RobotAssembly(body, cameras=None, end_effectors=None, embodiment=None, first_frame_timeout_s=3.0, microphones=None, speakers=None, first_audio_timeout_s=2.0, parts=None, placement=None, collision=None, own=None)`

`hirobot.assembly`. A body driver plus camera drivers, end-effector drivers and
audio devices; or, with `parts=` (`body=None`), a node of an assembly tree whose
children are robots of their own (see [Child parts](#child-parts) below).

- `cameras: Dict[str, CameraDriver]`: each camera's newest frame goes into
  `images[name]` with its stamp in `stamps[name]` (on a simulated body, restamped
  with the simulated time, `seq` kept). A camera that
  misses its first frame within `first_frame_timeout_s` only logs a warning; its
  key is absent until frames arrive.
- `end_effectors: Dict[str, EndEffectorDriver]`, by interface:
  - `joints` (hands): `embodiment` must then list the body joints plus every
    end-effector joint (`ValueError` otherwise). Joint vectors are gathered from
    body and end-effectors into one vector, and `joint_target` / `kp` / `kd` are
    split back by name. For joint modes, these end-effectors accept only
    `POSITION` and `PD`. When the body takes a non-joint mode (it balances
    itself), the action goes to the body and finger targets named in
    `joint_names` go to the end-effectors (each takes all its joints at once);
    `direct_joints` then includes every end-effector joint, and `hold()` leaves
    the body to hold itself and keeps each end-effector at its measured pose.
  - `opening` (grippers): `Action.gripper[<name>]` goes to the driver in any
    mode, after the safety gate's `gripper_range`, and is taken out of what the
    body gets; a value outside [0, 1] is refused (`ValueError`) before anything
    moves. The reading goes to `gripper[<name>]`. These restrict no mode, and
    `hold()` leaves them where they are.

  Either way, other signals (tactile arrays, contact) appear in `extra` as
  `<name>_<key>` and the reading's stamp in `stamps[<name>]`.
- `microphones: Dict[str, MicrophoneDriver]`, `speakers: Dict[str, SpeakerDriver]`:
  opened on `connect()` and closed on `disconnect()`; nothing goes into the
  observation. A microphone without audio within `first_audio_timeout_s` only logs
  a warning.
- `damp()`: damp the body (if its driver supports it) and every end-effector.
- `estop()`: e-stop the body and every end-effector, each cutting its own ramp short.
- `disconnect()`: disconnects the body first, then the end-effectors, then stops cameras
  and closes the audio devices.

### Child parts

With `parts=` (part name -> `Robot`) and `body=None`, a `RobotAssembly` is a
node of an assembly tree: several complete robots (two arms, a humanoid and an
arm) plus the node's own cameras, microphones and speakers, driven as one
`Robot`. `make_robot` builds it from an `assembly:` config with a `parts:`
section; parts nest, so a part may itself be an assembly with parts. The
behavior lives in `hirobot.assembly.node`:

```python
robot = make_robot("ur5e_dual")              # two ur_ur5e arms, left and right
robot = make_robot("examples/lab_two_cells.yaml")   # two cells of two arms each
```

| Topic | Behavior |
|---|---|
| Names | Every child part's joint, end-effector, camera, microphone and speaker is `<part>/<name>` in `embodiment`, the whole path for nested nodes (`cell_a/left/shoulder_pan`). The node's own body (`own=`), if any, keeps its names. Joints are concatenated: the own body first, then each child in order |
| Frames | `placement={part: {"pose": [...]}}` fixes where a child sits in the node; `{"origin": [...]}` is for a child that moves by itself (its odometry origin). `ee_poses` and `ee_targets` are in the node's frame: a fixed child is converted with its pose, a moving child with its origin composed with its current `base_position` / `base_orientation`. While a moving child reports no `base_position`, its `ee_poses` go to `extra["<part>/ee_poses"]` (own frame) and its end-effectors leave `cartesian_targets`. A child without a placement sits at the node's origin |
| `embodiment` | The merged tree (`hirobot.assembly.parts.merge_embodiments`): joints concatenated, `floating_base` when exactly one part has one, `control_hz` the node's. `embodiment.parts` maps each path to its joint span and children |
| `get_observation()` | Joint vectors concatenated; `ee_poses`, `gripper`, `images`, `depth`, `stamps`, `extra` keys prefixed (`stamps["left/body"]`). With exactly one floating-base part its `base_*` fields stay top-level; with several they go to `extra["<part>/base_*"]` |
| `send_action(action)` | Flat or per part. Per part (`Action.parts`): part name -> its own action in its own names and mode; a part's action may nest `parts` for a child node. Flat: one mode for every part, `joint_target` / `kp` / `kd` split by the parts' joint counts (or by name with `joint_names`), `ee_targets` / `gripper` by name prefix, `base_velocity` / `base_target` / `latent` / `motion` to the one floating-base part. A part the action does not mention gets nothing and keeps its last target; its simulation still advances one period (`idle_step`) |
| Checks | Every part's mode and `direct_joints` are checked on its piece, then the collision check (when configured); if any refuses, nothing is sent to any part. The one gate around the root checks the whole action (including `Action.parts`) in the robot's names, per joint |
| Capabilities | `action_modes`: modes every part takes. `direct_joints`, `cartesian_targets`: the parts', prefixed. `motion_spaces`: the floating-base part's when there is exactly one |
| Errors | A part that fails to read or send, or reports a `fault`, makes the assembly hold every other part and enter its own fault state. Then `send_action` raises `RobotFaultError` until `reset()` |
| Leases | A part is a `Robot` with its own [command lease](#command-arbiter), so `robot.claim(name, parts=["left"])` holds just that part (`parts=["cell_a/left"]` for a nested one); disjoint parts are claimed independently, overlapping paths raise `RobotBusyError`. A direct call on a held part raises `RobotBusyError`. A part's own thread (a whole-body controller) still commands its inner robot |
| `part(path)` | Read-only view: `embodiment`, `simulated`, `fault`, `get_observation()`, `get_status()` |
| `get_status()` | `joint_temperature` concatenated; `joint_faults`, `board_temperature`, `extra` keys prefixed; `battery` and `command` top-level when one part reports one, else `extra["<part>/battery"]` / `extra["<part>/command"]`; `fault` the assembly's or the first part's |
| Skills | Part skills listed as `<part>/<skill>`; `run_onboard_skill("g1/wave")` runs it on that part |
| Lifecycle | `connect` connects parts then the node's own cameras, undoing on failure. `reset`, `hold`, `damp`, `estop`, `disconnect`, `stop_onboard_skill` reach every part even when one fails, then raise the first error. `reset` also clears the fault state |
| Collision | Optional `collision=CollisionCheck(...)` (config: `collision:`). Each modelled part is capsules, one per body-to-parent segment of its MJCF (`hirobot.geometry.kinematics`), all of `radius`; only gaps between different parts count. Before sending, the assembly predicts each part's next pose (`POSITION` / `PD` joint targets as the next angles, otherwise the current ones; `CARTESIAN` target points as spheres) and refuses with `SafetyError` when the smallest gap would be under `min_distance`, unless it is not shrinking (parts already too close may move apart). Reads each modelled part's observation once per send. A floating-base part needs a base pose. About 1 ms for two G1s. Capsules along the chain are coarse: pick `radius` to cover the links; `skip:` leaves out bodies such as a mounting plate |
| Whole-body control part | A part wrapped in hiWBC's `wbc` layer (`layers: [{type: wbc, profile: ...}]` in its config) takes only `BASE_VELOCITY` / `MOTION_LATENT` / `MOTION_CLIP`, so the assembly's `action_modes` is usually empty: command it with `Action.parts`, joint modes for the arms and a walk command for it. Its free joints (`direct_joints`, e.g. the upper body) may ride along by name in the same sub-action. When it is not named, `idle_step` runs one controller tick, so a simulated humanoid keeps standing. A controller error faults the part and holds the assembly. hiWBC's own tests cover this with the `echo` profile |
| Example | `ur5e_dual`: two UR5e arms (`left`, `right`) ±0.4 m apart; `franka_ur5e_dual` adds a Franka FR3 and a `scene` camera; `examples/lab_two_cells.yaml` nests two `ur5e_dual` cells under one lab. All parts default to the toy driver, so they run without hardware (override each arm's body driver by its path, `overrides={"left/body": {"driver": ...}}`) |

Collision check `collision:` section (paths relative to the config file):

```yaml
collision:
  min_distance: 0.05        # m between capsule surfaces (default 0.05)
  radius: 0.05              # default capsule radius (m)
  backend: numpy            # or mujoco, for MJCFs the numpy parser rejects
  models:                   # parts without a model are not checked
    left:  {mjcf: ~/models/fr3.xml}
    right: {mjcf: ~/models/ur5e.xml, radius: 0.06, skip: [base_mount]}
```

## Control loops

hiRobot has no policy classes, control loop or model-server client; it only
supplies the `Robot` a loop drives. What a loop uses from a driver:
`reset(seed=None)`, `get_observation()`, `send_action()`, `hold()` after a stop,
`embodiment.control_hz` for pacing a real robot, `simulated` (step a simulator
unpaced), and on simulators `success` / `done` / `reward`.
`ActionChunk.step(i)` turns one step of a predicted chunk into an `Action`.

## DofMap

`hirobot.core.dof.DofMap(source_names, target_names)` maps between two joint orderings
by name. Raises `ValueError` if a target name is missing from the source.

| Method | Description |
|---|---|
| `remap(values)` | Source-ordered array → target order (first axis). Checks the length |
| `scatter(values, default=None)` | Target-ordered array → source order. Source joints not in the target take `default` (or `0`). Use it when a policy commands a subset of joints |

```python
m = DofMap(robot.embodiment.joint_names, policy_joint_names)
policy_q = m.remap(obs.joint_pos)
robot_target = m.scatter(policy_target, default=obs.joint_pos)
```

## Features

`hirobot.core.features` turns an `Observation` or `Action` into flat float32
vectors with a name for every dimension, the layout a dataset such as
LeRobot's stores. Joints are in `Embodiment.joints` order; an assembly's names
carry the `<part>/` prefix.

```python
from hirobot import flatten_observation, observation_features

names = observation_features(robot.embodiment)   # {"observation.state": ["j0.pos", ...], ...}
obs = robot.get_observation()
row = flatten_observation(obs, robot.embodiment)  # {"observation.state": array([...]), ...}
```

| Key | Values | Names |
|---|---|---|
| `observation.state` | `joint_pos`, then `gripper` of each end-effector with a gripper | `<joint>.pos`, `<ee>.gripper` |
| `observation.ee_pose` | `ee_poses`, 7 per end-effector | `<ee>.x` ... `<ee>.qw` |
| `observation.joint_vel`, `observation.joint_torque` | `joint_vel`, `joint_torque` | `<joint>.vel`, `<joint>.torque` |
| `observation.imu` | `base_orientation` `[x, y, z, w]`, `base_angular_velocity`, `base_linear_acceleration` | `imu.qx` ... `imu.az` (10) |
| `observation.base` | `base_position`, `base_linear_velocity` | `base.x` ... `base.vz` (6) |
| `action` | Joint modes: `joint_target`, then grippers. Beside a non-joint mode: the targets `joint_names` gives, every joint, `NaN` for the others | `<joint>.pos` (`.vel`, `.torque` for those modes), `<ee>.gripper` |
| `action.gripper` | Grippers under a non-joint mode | `<ee>.gripper` |
| `action.ee`, `action.ee_delta` | `ee_targets` under `CARTESIAN` (7 each) / `CARTESIAN_DELTA` (6 each) | `<ee>.x` ... `<ee>.qw` / `<ee>.dx` ... `<ee>.daz` |
| `action.base_velocity`, `action.base_target` | `base_velocity`, `base_target` (`BASE_POSE`) | `base.vx`, `base.vy`, `base.wz` / `base.x`, `base.y`, `base.yaw` |

| Function | Returns |
|---|---|
| `observation_features(embodiment, obs=None)` | Key -> names. Without `obs`, every key the embodiment can have (`imu` and `base` only with a floating base); with `obs`, the keys it fills |
| `flatten_observation(obs, embodiment)` | Key -> vector. `ValueError` on a wrong length |
| `action_features(embodiment, mode=POSITION, action=None)` | Key -> names, for `mode`, or for `action` |
| `flatten_action(action, embodiment)` | Key -> vector. `Action.parts` is flattened in the robot's names; its leaves must share one mode |
| `image_features(embodiment)` | `observation.images.<cam>` -> camera name. Images are not flattened |

A key whose source the observation or action does not fill is left out. A key
that is present always has its full length; a missing piece inside it (one
end-effector's pose, the IMU's acceleration, a gripper) is `NaN`. `kp` / `kd`,
`latent`, `motion` and `extra` are not flattened.

## Recording

`hirobot.record` saves episodes as LeRobot v3.0 datasets without importing
lerobot (`pip install 'hirobot[record]'`: pyarrow and PyAV). A `Recorder` is a
[listener](#listeners) on the outermost robot: it never commands the robot, and
an error in it never reaches the control loop.

```python
from hirobot.record import Recorder

rec = Recorder(robot, "data/")           # fps: min(30, control_hz), control_hz without cameras
rec.subscribe(lambda e: print(e.state, e.path, [f.name for f in e.flags]))
rec.start("pick the cube", meta={"operator": "ann"})
...                                      # the control loop runs as usual
rec.stop()                               # returns at once; the episode saves in the background
rec.close()                              # saves what is queued
```

| Item | Meaning |
|---|---|
| `Recorder(robot, root, *, fps=None, max_queue=15, min_duration_s=1.0, max_buffer_mb=4096, max_samples=1_000_000, copy_images=False, robot_config=None, hold_after_s=0.5, frozen_s=1.0, max_queue_mb=8192, worker="process", ring_mb=256)` | `ImportError` without pyarrow or PyAV. `fps` must be a whole number. Past `max_buffer_mb` of image pixels or `max_samples` readings an episode keeps no new ones. `copy_images` is accepted but does nothing: every kept image is copied into the frame ring anyway |
| `worker` | Where episodes are kept and saved. `"process"` (default): a child Python process, started on the first `start()` and reused, so aligning and encoding never hold the control process's GIL. `"thread"`: a thread of this process. Without `memfd` (not Linux), or when the child exits twice in a row before it is ready, the recorder logs a warning and switches to `"thread"` |
| `ring_mb` | Size of the shared-memory ring new images are copied into on their way to the worker (at least 16 frames of the largest declared camera), allocated and touched once in the constructor. A frame that arrives when the ring is full is dropped and the episode is flagged `dropped_samples` |
| `Recorder.worker` | `"process"` or `"thread"`: where episodes are saved now, after any fallback |
| `start(task, meta=None) -> int` | Starts an episode and returns its number. `RecorderError` when already recording, or when the stopped episodes still waiting to be saved number `max_queue` or hold more than `max_queue_mb` of images. `meta` is stored as is |
| `stop() -> int` / `cancel()` | `stop` queues the episode for saving; `cancel` drops it |
| `subscribe(callback) -> Subscription` | `callback(SaveEvent)` on the recorder's dispatcher thread (never the control loop's or the worker's): `saving`, then `saved` (with `path`, `flags`, `frames`) or `failed` (with `error`). A slow callback delays later events, not recording |
| `wait(timeout=None)`, `close(timeout=None)`, `recording`, `queue_length` | `close` stops an open episode and saves it (`"stopped_by": "close"` in its metadata), ends the worker, delivers the last events, then ends the recorder's threads and frees the frame ring. After `timeout` a child process still saves the episodes it already received, even after this process exits; samples not yet passed to it are lost. A recorder dropped without `close()` is garbage-collected and its threads and worker end the same way |
| `append_verdict(path, episode, verdict, note="", reviewer=None)` | Adds a reviewer's `PASS` / `FAIL` / `UNREVIEWED` line to `meta/qc.jsonl`; `read_qc(path)` reads the lines back |

While recording, the listener callbacks keep one state sample per new
`stamps[source].seq` (without `images`, `depth` and `extra`, which nothing in
the dataset reads) and copy one image per new camera `seq` into the frame ring;
a sender thread streams both to the worker. After `stop()` the worker aligns everything to a grid of `1/fps` over the time all streams cover
(continuous values interpolated linearly, quaternions normalized, grippers take
the last reading at or before the frame, commands are paired with the state
reading they followed, images take the nearest frame), encodes each camera as
H.264 and writes one directory per episode:

```
<root>/<YYYYmmdd-HHMMSS>_<task>_<NNNN>/
  meta/info.json, meta/stats.json, meta/tasks.parquet, meta/episodes/chunk-000/file-000.parquet
  data/chunk-000/file-000.parquet
  videos/observation.images.<cam>/chunk-000/file-000.mp4
  meta/hi/info.json           # hi.action_mode, hi.robot_config, hi.hirobot_version, hi.control_hz, hi.recorder_fps
  meta/hi/alignment.json      # streams, image offsets, trimmed time, commands, lease events
  meta/hi/episode.json        # task, meta, start and stop times
  meta/qc.jsonl               # the save-time flags, then any reviewer verdicts
```

`meta/info.json` holds only LeRobot's own keys. The recorder's dataset-level
keys are in `meta/hi/info.json`, one flat JSON object whose keys keep the `hi.`
prefix (LeRobot drops unknown keys of `info.json` when it loads a dataset).

The state and action columns are the [features](#features) (`observation.state`,
`action` and the rest). The recorder adds:

| Column | Meaning |
|---|---|
| `action` | The commands of the episode's most common mode (`hi.action_mode` in `meta/hi/info.json`). Row `k` holds the first command sent after the frame's state reading and before the next reading, so row `k` is the state the command answered; with no command there, the one still in force. When readings come slower than the grid (a loop that observes once per action chunk), later frames showing the same reading take the first command sent in their own frame period, else the one in force at the frame time. After a `hold()`, and in frames counted as `hold` because no command came for `hold_after_s` (the command watchdog's hold is not reported), position-type targets (`POSITION`, `PD`, `CARTESIAN`, `BASE_POSE`) and grippers keep their last value, while velocities, per-tick deltas and torques (`VELOCITY`, `TORQUE`, `CARTESIAN_DELTA`, `BASE_VELOCITY`, and `base_velocity` beside `CARTESIAN`) read 0. Frames under a command of another mode are `NaN` and the episode is flagged `mixed_action_modes` |
| `hi.control_source` | The source of that command: `policy` (a direct call or a `CALLER` lease), `teleop` (a `TAKEOVER` lease), `skill` (`LAYER_SKILL`), or `hold`: no command yet, a `hold()`, or none for `hold_after_s` |
| `intervention` | `hi.control_source == "teleop"` in an episode that also has `policy` frames; a pure teleoperation episode is all `false` |
| `hi.rejected_action`, `hi.rejected_action_mask` | A refused `send_action` (preempted, refused by an envelope or the gate) in the one frame whose span holds it, the last one when a span has several. A frame's span starts at its state reading, or at its grid time when an earlier frame already shows that reading, and ends where the next frame's starts. Where there was none, `hi.rejected_action` is `0.0` and the mask `false`; `0.0` also where a refused action could not be converted to the columns (the mask is still `true`). Use the mask, never the values, to tell which rows hold one. Zeros, not `NaN`, because pandas (and so LeRobot's `aggregate_datasets`) turns `NaN` inside a list column into null |
| `hi.capture_time` | The robot-clock time of the state reading at or before the frame |
| `hi.stamp.<source>.t`, `hi.stamp.<source>.seq` | That reading's stamp, for every source, cameras included |
| `hi.image_skew.<cam>` | Image time minus frame time, in seconds |
| `hi.ik_joint_target` | The joint targets an IK layer sent for the frame's command (when the robot has `commanded_joint_targets`) |

Save-time flags, in the `SaveEvent` and in `meta/qc.jsonl` (verdict `FAIL` when
any): `too_short`, `missing_camera`, `camera_gap`, `frozen_camera` (the 32x24
picture did not change for `frozen_s` while frames arrived), `alignment` (more
than 1% of frames take an image further than `1/(2 fps)` s away),
`invalid_state`, `dropped_samples` (a buffer limit reached, the frame ring
full, or a sample the worker could not take), `fault` (the
robot reports a fault at `stop()` that it did not have at `start()`),
`no_commands`, `mixed_action_modes`, `unrecorded_action`.

The worker holds the raw images of the episode being recorded and of the
stopped episodes not yet saved, about 6 MB per 1080p frame. One episode keeps
at most `max_buffer_mb` of them. `start()` refuses while the stopped episodes
not yet saved hold more than `max_queue_mb`, so the raw images stay under
`max_queue_mb + 2 * max_buffer_mb` (16 GB with the defaults). The control
process itself holds only the frame ring (`ring_mb`).

In a simulation (`toy`, `mujoco`) time advances only when a command is sent or
an assembly idles a part, so after `hold()` a loop that only calls
`get_observation()` reads the same simulated instant again and again. Those
reads are de-duplicated by their stamp, no frames past `hold_after_s` appear,
and the episode ends at the last command. To record a hold in simulation, keep
sending: call `hold()` (or resend the held target) every tick. On a real robot
the clock runs by itself and no extra calls are needed.

MuJoCo episodes recorded before this version have `NaN` in `observation.imu`
dims 7 to 9 (linear acceleration); don't merge them with newer episodes.

**Known limitation: control-loop latency with large images.** Measured on a
50 Hz MuJoCo G1 loop with IK and one 1920x1080 camera, the simulator rendering
on an idle GPU:

- While recording, the tick p99 is 8.2 to 8.9 ms and the worst tick 14.3 ms;
  no tick took over 20 ms.
- While an episode saves, the tick p99 is 8.4 to 10.2 ms.
- After `stop()` the control process spends about 2 ms of CPU per episode
  (about 600 ms when episodes were saved on a thread of this process).

On a GPU that also drives a display, the simulator's synchronous 1080p readback
alone reached 23.7 ms p99 without recording. Choose an idle GPU for MuJoCo's EGL
rendering with `MUJOCO_EGL_DEVICE_ID`. Setting `NUMPY_MADVISE_HUGEPAGE=0` before
numpy is imported avoids transparent-hugepage compaction stalls when large frame
arrays are allocated.

## Kinematics

`hirobot.geometry.kinematics.Kinematics.from_mjcf(path, backend="numpy")` loads an MJCF model and
computes world poses of every body from joint angles and the base pose. Positions are in
metres, quaternions `[x, y, z, w]` with `w >= 0`.

| Backend | Needs | Notes |
|---|---|---|
| `"numpy"` (default) | numpy only | Reads the body tree from the MJCF: nested `<body>` with `pos` / `quat`, hinge joints (`axis`, `pos`), one free joint on a top-level body. Raises `ValueError` for MJCF features it does not read (`euler` / `axisangle` / `xyaxes` / `zaxis`, slide / ball joints, joint `ref`, `<default>` joint attributes, `<include>`, `<frame>`) |
| `"mujoco"` | `pip install 'hirobot[sim]'` | `mujoco.mj_kinematics` on the same file. Raises `ImportError` without mujoco. Use it to cross-check the numpy result or for models the numpy parser rejects |

The backend is chosen explicitly, never by what is installed. The tests check the numpy
backend against `mj_kinematics` (G1 29 DoF: max difference below 1e-12).

| Member | Description |
|---|---|
| `body_names`, `parents` | Bodies in model order (world body excluded) and their parent index (`-1` = top level) |
| `joint_names` | Hinge joints in model order, as named in the MJCF |
| `floating_base` | True when a top-level body has a free joint |
| `forward(joint_pos, base_pos=None, base_quat=None, joint_names=None) -> BodyPoses` | `joint_pos` is `(..., n)`. With `joint_names` (`name` or `name_joint`), columns are matched by name and model joints not listed are at 0; without it, columns follow `Kinematics.joint_names`. `base_pos` / `base_quat` place the free-joint body (default: its MJCF pose; floating-base models only) |

`BodyPoses` has `names`, `pos` `(..., num_bodies, 3)` and `quat` `(..., num_bodies, 4)`;
`poses["left_wrist_yaw_link"]` returns that body's `(pos, quat)`.

```python
from hirobot.geometry.kinematics import Kinematics

kin = Kinematics.from_mjcf("~/.cache/hirobot/assets/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")
poses = kin.forward(obs.joint_pos, base_pos=[0, 0, 0.79], base_quat=[0, 0, 0, 1],
                    joint_names=robot.embodiment.joint_names)
pos, quat = poses["torso_link"]
```

## Simulation assets

`hirobot.assets` downloads simulation models (MJCF / URDF) from the Hugging Face dataset
`humanoidintelligence/hirobot-assets` into `~/.cache/hirobot/assets`, keeping the dataset's
layout; the built-in `*_mujoco` configs name files there. Needs `pip install
'hirobot[assets]'` (`huggingface_hub`). Nothing goes to the network on import or when a
config loads, only in `fetch`. `hirobot assets list` / `hirobot assets fetch` are the CLI.

| Name | Description |
|---|---|
| `REPO_ID`, `REVISION` | The dataset and the tag of it that `fetch` downloads |
| `repo_id() -> str` | The dataset `fetch` uses: `$HIROBOT_ASSETS_REPO` (another dataset with the same layout and tags, such as a mirror) if set, else `REPO_ID` |
| `ASSETS` | `{name: allow_patterns}`: each model's directories plus its source's `LICENSE` and `SOURCE.md` and the dataset's `README.md` |
| `assets_dir() -> Path` | `~/.cache/hirobot/assets`, where the built-in configs look |
| `fetch(names, *, token=None) -> Path` | Download one name or a list (`snapshot_download`, files already there are kept) and return `assets_dir()`. `ValueError` for an unknown name, `RuntimeError` without `huggingface_hub`. `token` defaults to the saved `hf auth login` or `$HF_TOKEN` |
| `is_fetched(name) -> bool` | Whether that model's files are in `assets_dir()` |
| `asset_of(path) -> Optional[str]` | The name whose model files include `path` (under `assets_dir()`), else `None` |

```python
from hirobot import assets

assets.fetch(["unitree_g1", "menagerie_universal_robots_ur5e"])
assets.is_fetched("unitree_g1")   # True
```

## Config file reference

A YAML file read by `load_embodiment` / `make_robot` / the CLI, given as a
built-in name (`"unitree_g1"`, `"body/unitree/h1"`; list them with
`hirobot configs`) or a file path. A built-in name is looked up as `configs/<name>.yaml` in the package, then
`configs/robots/<name>.yaml`, so `"unitree_g1"` and `"robots/unitree_g1"` (the form
`hirobot configs` lists) name the same file. A string ending in `.yaml` / `.yml`
or starting with `/`, `~` or `.` is a path. A config can list others under `defaults:`;
they are merged in order and the file itself last. Mappings merge key by key;
`joints`, `frames`, `mounts` and the device lists merge by `name` (same name:
fields merged; new name: appended); any other value is replaced. A device's
`driver:` merges key by key while its `type` stays the same and is replaced
whole when `type` changes. `hirobot info --config <name> --resolved` prints the
result. Rules and examples: `hirobot.loading.config`, `hirobot.loading.tree`.

A config is one of two shapes. A **device file** describes one device model; its
top level is the `device:` kind (`body`, `end_effector`, `camera`, `microphone`
or `speaker`):

```yaml
device: body                # the kind; the other keys depend on it
control_hz: 125
floating_base: false
joints:                     # body / end_effector
  - name: shoulder_pan
  - {name: wrist_3, position_limit: [-6.28, 6.28], torque_limit: 28}
frames:                     # task-space frames (end-effector sites)
  - {name: tcp}
mounts:                     # where components may hang (link -> site)
  wrist_camera: {link: wrist_3_link}
model:                      # optional: the body's model file (Embodiment.model)
  mjcf: ~/models/ur5e.xml   # MJCF, for sim / collision / kinematics
  # urdf: ../urdf/ur5e.urdf                        # or a URDF (drawing only)
  # package_roots: {ur_description: ../urdf}       # URDF package:// roots
driver: {type: ur_rtde}     # registered driver name + its constructor kwargs
```

`model:` needs `mjcf` or `urdf`; with both, `Embodiment.model` gives the MJCF. Relative paths (also in `package_roots`) are taken from the device file's directory. The simulation, the `mujoco` driver and `mink_ik` need `mjcf`; a body with only a `urdf` is refused there. `kp`, `kd`, `armature`, `home` set the `mujoco` body's gains, armature and start pose.

A camera file carries `width` / `height` / `fps` / `depth` / `fovy` /
`intrinsics` (`{fx, fy, cx, cy}`, optional `width`, `height`, `model`, `coeffs`); an
end-effector file `interface` (`joints` or `opening`), `joints`, `tcp`; a
microphone or speaker only `driver`.

A **robot config** has an `assembly:` node listing its parts. A part is a device
or another node, referenced with `from:` (a built-in name or a path, relative to
the file that names it) or written inline (`device:` or `assembly:`):

```yaml
assembly:
  name: ur5e_dual
  control_hz: 30            # optional; default: the slowest part's
  parts:                    # name -> part; joint order follows this order
    left:  {from: robots/ur_ur5e, overrides: {body: {driver: {ip: 192.168.1.101}}}}
    right: {from: robots/ur_ur5e, safety: {max_joint_step: 0.1}}   # tightens this part only
    scene: {device: camera, width: 640, height: 480, driver: {type: array}}   # inline device
  placement:               # where each part sits in the node
    left:  {pose: [0, 0.4, 0, 0, 0, 0, 1]}         # fixed: [x, y, z, qx, qy, qz, qw]
    right: {pose: [0, -0.4, 0, 0, 0, 0, 1]}
    scene: {on: body.wrist_camera, pose: [0, -0.07, 0.03, 0, 0, 0, 1]}  # on a part's mount
    # a part that moves by itself uses origin: instead of pose:
    # rover: {origin: [0, 0, 0, 0, 0, 0, 1]}
  collision: {min_distance: 0.05}                  # optional, see RobotAssembly

# Sections only the root config may carry (a referenced part cannot):
safety:                    # optional; `safety: false` detaches the gate
  max_joint_step: 0.2      # SafetyGate kwargs (joint_limit, clamp, gripper_range, ...)
layers:                    # optional; registered layers, first innermost
  - {type: my_wbc, part: left}    # part: wraps only that part (else the whole robot)
arbiter:
  require_claim: false     # optional; refuse commands without a lease (see Command arbiter)
  audit_size: 256          # optional; events the audit ring keeps
sim:                       # optional; make_robot(..., sim=True) builds a shared MuJoCo world
  realtime: true
  camera_fps: 30           # cameras without fps; null renders every observation
mcp:                       # optional; what `hirobot mcp` / serve_mcp serves (see MCP server)
  tools: [observe]
```

A single-robot config is an `assembly:` with one `body` part (see
`robots/ur_ur5e`). `overrides` on a device reference is merged into the device
file; on a node reference it maps a child part's name to that part's overrides,
level by level. `make_robot(cfg, overrides={"left/body": {"driver": {"ip": ...}}})`
merges host-specific values the same way from the command line.

Policy settings (model server, chunk length) are not part of a robot config.

## CLI

Installed as `hirobot`.

| Command | Description |
|---|---|
| `hirobot --version` | Print the version |
| `hirobot info --config NAME_OR_PATH [--resolved]` | Print the embodiment: kind, control rate, floating base, joints, end-effectors, cameras, model file. `--resolved` prints the whole config after `defaults:` are merged |
| `hirobot configs [GROUP]` | List the built-in configs (`robots`, `body`, `end_effector`, `camera`, `microphone`, `speaker`). Names under `robots/` also work without the prefix (`unitree_g1`) |
| `hirobot assets list` | List the simulation models `hirobot assets fetch` can download, whether each is in `~/.cache/hirobot/assets`, and what it is (see [Simulation assets](#simulation-assets)) |
| `hirobot assets fetch NAME... \| --all [--token T]` | Download those models (or all) into `~/.cache/hirobot/assets`; needs `hirobot[assets]`. Exit 2 for an unknown name, without `huggingface_hub`, or when the download fails (a private dataset needs `hf auth login`; `HF_ENDPOINT` can point at a mirror) |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | Run a simulated robot: connect, hold the start pose for N s (default 3), print the joint error left, the base height, the rendered images and `get_observation` time. Says when a free base fell. Exit 2 for a real body; `--viewer` needs a MuJoCo body |
| `hirobot check --sim CONFIG [--against-real] [--driver PART:KEY=VALUE ...] [--tolerance 0.02]` | Build the world the simulation check uses, without stepping it: every body needs a `model:`, every joint must be in its MJCF with an actuator, every end-effector frame a body of it; joint limits that differ from the MJCF's range by more than 0.1 rad are printed as warnings. `--against-real` also opens every body's real driver `read_only: true` (refused without that mode), reads one observation and prints each end effector's measured vs model position at those joint angles. Exit 1 on an error or an error above `--tolerance` m. Library: `hirobot.sim.check_sim` |
| `hirobot mcp CONFIG [--transport stdio\|http] [--host H] [--port P] [--path /mcp] [--tools CATEGORY ...] [--max-steps N] [--no-reset]` | Serve the robot as an MCP server (see [MCP server](#mcp-server)); needs `hirobot[mcp]`. Exit 1 without the `mcp` package or on a bad config |
| `hirobot view CONFIG [--model MUJOCO_CONFIG] [--body KEY=VALUE ...] [--hz 30] [--seconds N]` | Show a real robot's pose in a MuJoCo viewer. Opens CONFIG with `read_only: true` (a driver without a `read_only` mode is refused), copies the measured joints (by name) and base orientation into the model each frame, and never sends a command. `--model` defaults to the built-in `<config>_mujoco`. Exit 2 for a simulated robot or no model. Library: `hirobot.sim.view` |
| `hirobot scan [--interface NIC] [--timeout 2] [--no-unitree] [--json]` | Look at this computer and the robot network, read-only, and print a summary (`--json`: the report only). Sends no command and changes no setting. Exit 0 even when nothing is found. See [`hirobot scan` and `hirobot init`](#hirobot-scan-and-hirobot-init). Library: `hirobot.scan` |
| `hirobot init -o FILE [--model NAME] [--interface NIC] [--answers FILE] [--yes] [--force] [--no-verify] [--timeout 2]` | Scan, pick the model and network card, write a local config to FILE, then open it read-only and say which parts sent data. Exit 0 written (and checked), 1 nothing written, 2 written but the check failed. Library: `hirobot.scan.init_config` |

### `hirobot scan` and `hirobot init`

This version knows Unitree robots only (G1, G1 with Dex3 hands, H1, H1 with Inspire
hands, H1-2, H2). `hirobot scan` reads the host (network cards, installed SDKs, ROS 2,
cameras, audio devices) and, when a network card has an address on `192.168.123.x` (or
`--interface` names one), probes the Unitree robot on it. The probe only subscribes to
`rt/lowstate` and `rt/dex3/{left,right}/state` and asks the motion switcher `CheckMode`
once. It creates no DDS publisher, sends no command and changes no network setting.
Anything that needs `sudo` is printed under suggestions and not run.

On a G1 with Dex3 hands, `hirobot scan --json` prints a report like this (example,
shortened):

```json
{
  "version": 1,
  "host": {
    "hostname": "robot-pc", "os": "Linux 5.15.0-tegra", "arch": "aarch64",
    "python": "3.10.12", "jetson": "NVIDIA Jetson Orin NX",
    "interfaces": [{"name": "eth0", "up": true, "ipv4": ["192.168.123.164/24"],
                    "mac": "00:11:22:33:44:55", "speed_mbps": 1000, "unitree_subnet": true}],
    "sdks": [{"distribution": "unitree_sdk2py", "minimum": "1.0.1", "installed": "1.0.1",
              "ok": true, "drivers": ["unitree_handcmd", "unitree_sdk2", "..."],
              "install": "pip install -e <clone of https://github.com/unitreerobotics/unitree_sdk2_python>"}],
    "ros2": {"distro": null, "rclpy": false},
    "cameras": [{"kind": "realsense", "name": "Intel RealSense D435", "serial": "123456789",
                 "device": null}],
    "audio": {"source": "alsa", "devices": [{"name": "USB Audio", "inputs": 1, "outputs": 2}]},
    "suggestions": [],
    "notes": []
  },
  "unitree": {
    "interface": "eth0",
    "status": "found",
    "error": null,
    "msg_family": "hg",
    "motor_slots": 35,
    "motors_reporting": 29,
    "reporting_slots": [0, 1, 2, "...", 28],
    "mode_machine": 5,
    "lowstate_hz": 500.0,
    "motion_service": {"checked": true, "mode": "ai", "owns_lowcmd": true},
    "hands": {"dex3_left": true, "dex3_right": true},
    "candidates": [
      {"config": "robots/unitree_g1_dex3", "confidence": 0.9, "why": "unitree_hg with 29 motors reporting: G1 29-DoF; rt/dex3/left/state and rt/dex3/right/state seen"},
      {"config": "robots/unitree_g1", "confidence": 0.85, "why": "unitree_hg with 29 motors reporting: G1 29-DoF"}
    ],
    "notes": ["built-in motion service 'ai' owns rt/lowcmd; joint commands need it released (robot hung or supported)"]
  }
}
```

A value the scan could not see is `null` (or `[]`): it means "not seen, please
confirm", never "absent". A G1 with a locked waist reports 27 motors, and a Dex3 hand
that is off the bus shows as `null`.

| Key | Meaning |
|---|---|
| `host.interfaces[].unitree_subnet` | The card has an IPv4 address on `192.168.123.0/24`; the first such card that is up is probed |
| `host.sdks[]` | One entry per SDK a driver manifest lists. `installed` is the version, `"unknown"` when the module imports but has no package metadata (a source checkout), `null` when it is not installed. `ok` is true when installed and not older than `minimum`. `install` is the manifest's install hint |
| `host.suggestions` | Commands for you to run (an address for the robot's subnet, the SDK install); never run by the scan |
| `host.notes` | Probes that failed on this host (a missing `ip` command, a camera library that raised); the scan goes on |
| `unitree.status` | `found` (`rt/lowstate` arrived), `not_seen` (nothing arrived within `--timeout`), `skipped` (no network card on `192.168.123.x`, or `--no-unitree`), `error` (SDK missing, DDS could not start on the card; the text is in `error`) |
| `unitree.msg_family` | `hg` (G1, H1-2, H2) or `go` (the 19-DoF H1) |
| `unitree.motor_slots`, `motors_reporting`, `reporting_slots` | Motor slots in the `LowState` message, how many read a temperature or voltage, and their indexes. `reporting_slots` is only present when the probe ran |
| `unitree.motion_service` | `CheckMode` result: `mode` is the running built-in service (`""` when none); `owns_lowcmd` is true while one runs. `checked: false` when the query got no answer |
| `unitree.hands` | `true` when that Dex3 state topic sent data, `null` when not seen |
| `unitree.candidates` | Built-in robot configs that fit, highest `confidence` first, each with `why` |

`hirobot init` runs the scan, then:

1. **Model.** `--model` (or `model:` in the answers file) is used as given (`unitree_g1`
   and `robots/unitree_g1` both work). Without it, `init` lists the candidates and asks;
   `--yes` takes the first one; without a terminal it takes the first only when its
   confidence is 0.9 or more. No candidate and no model: nothing is written, exit 1.
2. **Network card.** `--interface`, `interface:` in the answers file, or the scanned
   card. None: exit 1.
3. **Existing file.** `init` asks before replacing FILE (default no). It overwrites only
   on a "y" answer, `--force` or `overwrite: true`. `--yes` alone never overwrites: the
   file is kept and the exit code is 1.
4. **Write.** FILE gets a header comment (model, card, date, what the scan saw), then
   `defaults: [robots/<model>]` and `assembly: parts:` overrides that set the body driver
   to `unitree_sdk2` with `network_interface`, and `network_interface` on every part whose
   driver takes one (Dex3 hands, head camera, microphone, speaker). No gains and no safety
   values: those stay the built-in defaults. The file is loaded with `load_embodiment`
   right away.
5. **Read-only check** (skip with `--no-verify` or `verify: false`). `init` opens FILE
   with the body driver `read_only: true` (as `hirobot view` does; hands become
   read-only with the body), reads observations for up to `--timeout` s (at least 1 s),
   and prints each part as data seen, partly seen, not seen or skipped. Cameras and
   microphones only receive, so they are opened. A part whose driver has no `read_only`
   mode is skipped and not opened: the speaker is one (it sets its volume on connect).
   The check fails when the robot does not connect, `joint_pos` does not have one
   value per joint, or a body joint is not finite. A hand, camera or microphone with no
   data, or one that fails to connect, is reported as not seen and left out without
   failing; only the body must report.

The answers file (`--answers FILE`) is YAML with these keys only, for runs without a
terminal:

```yaml
model: robots/unitree_g1_dex3   # skip the model question
interface: eth0                 # skip the network card choice
overwrite: true                 # replace FILE if it exists (default: keep it)
verify: false                   # skip the read-only check
```

Exit codes: 0 written (and the check passed or was skipped); 1 nothing written (no
model, no card, the file exists and was kept, a bad answer or model name); 2 written
but the read-only check failed (the file is kept, fix the cause and run `init` again).

Check the written file before the first commanded motion: see the
[Unitree setup guide](setup/unitree.md#generate-a-local-config).

## MCP server

How to connect a client, a first run in simulation and safe use on a real robot: [MCP guide](./mcp.md).

`pip install -e '.[mcp]'` (the `mcp` SDK, Python 3.10+, and Pillow) serves a
robot as an [MCP](https://modelcontextprotocol.io) server, so an LLM agent or
any MCP client can call its observation and command tools. The tools are the
robot API below them, nothing more: every command goes through the robot's
`send_action`, its `SafetyGate`, and a command lease. The control loop stays
with whoever calls the tools.

```bash
hirobot mcp robots/ur_ur5e                       # stdio: the MCP client starts this command
hirobot mcp my_robot.yaml --tools observe act    # also allow motion
hirobot mcp my_robot.yaml --transport http --port 8000   # streamable HTTP at http://127.0.0.1:8000/mcp
```

`hirobot mcp` builds the robot, connects, resets it (unless `--no-reset`),
serves until the client disconnects or Ctrl-C, then disconnects. Logs go to
stderr. HTTP listens on `127.0.0.1` by default; any other `--host` lets
everyone who can reach that port command the robot, and logs a warning.

### Tools

| Category | Tool | What it does |
|---|---|---|
| `observe` | `describe_robot` | Name, kind, `control_hz`, joints (names and limits, in the order joint lists use), end effectors, cameras with intrinsics and mount, `action_modes`, the tools that are on, `max_steps` |
| `observe` | `get_observation` | State as JSON (joint arrays as `{joint: value}`, end-effector poses, grippers, base, stamps) plus one JPEG per camera. `images: false` for state only; `cameras: [...]` to pick |
| `observe` | `get_robot_status` | `get_status()` (faults, battery, temperatures, gate counts, command progress) plus `skill_running` and the MCP lease state |
| `act` | `send_actions` | 1 to `max_steps` actions, one per control tick at `control_hz` (simulators too), then `hold`. A refused step ends the chunk. Returns `sent`, `of`, `stopped` (`done` / `refused` / `stop`), `error`, and the new observation |
| `act` | `hold` | `hold()` |
| `act` | `stop` | Interrupts a `send_actions` in progress before its next step, stops a running skill, holds. Not an emergency stop |
| `skill` | `list_skills` | `list_robot_skills()` with each skill's argument schema |
| `skill` | `run_skill` | `run_robot_skill(name, args)`; returns once it has started |
| `skill` | `stop_skill` | `stop_robot_skill()` |
| `episode` | `reset` | `reset(seed)`, then the new observation |

Only `observe` is on by default; motion must be enabled in the config or with
`--tools`. There is no `estop` tool (an e-stop is for a person or a watchdog,
not for a model to call) and no task-completion tool (that belongs to the
agent or the benchmark). Depth images are not sent.

An action in `send_actions` is an `Action` as JSON: `mode` (one of
`action_modes`), and the fields that mode uses. `joint_target` / `kp` / `kd`
take a list in `describe_robot` order or `{joint name: value}`; a mapping that
names only some joints sets `joint_names` (for the non-joint modes). `parts`
takes `{part: action}` on a robot with parts.

The first command claims a lease named `mcp` (see [Command arbiter](#command-arbiter)),
with the config's `envelope` if it sets one. If someone with a higher priority
takes the robot (`Priority.TAKEOVER`), the next command reports the
preemption and is refused; it does not claim again.

A tool the caller got wrong, or that the robot refused, comes back as a result
with `isError` set and the reason as text. Results also carry the JSON as
`structuredContent`.

### `mcp:` config section

Root config only, like `arbiter:`. Every key is optional; the command-line
options and `serve_mcp` keyword arguments win over it.

```yaml
mcp:
  tools: [observe, act]          # categories: observe, act, skill, episode (default [observe])
  max_steps: 10                  # most actions per send_actions call, 1-50 (default 5)
  image_max_side: 512            # longest side of a returned image, px (default 512)
  image_quality: 80              # JPEG quality 1-100 (default 80)
  envelope: {max_joint_step: 0.05}   # the mcp lease's limits; same keys as claim(envelope=...)
```

### From Python

```python
from hirobot import make_robot
from hirobot.mcp import serve_mcp

robot = make_robot("my_robot.yaml")
robot.connect()
serve_mcp(robot, transport="stdio", tools=["observe", "act"])   # blocks
```

`serve_mcp(robot, *, transport="stdio", host="127.0.0.1", port=8000, path="/mcp",
name=None, tools=None, max_steps=None, image_max_side=None, image_quality=None,
envelope=None)` does not connect or disconnect the robot; it releases the lease
on return. `hirobot.mcp.RobotTools(robot, settings=None)` is the same tools
without the SDK: `tools.call(name, args)` returns a `ToolResult` (`data`,
`images`). `build_server(tools)` returns the SDK's low-level `Server`, for
in-process use (`mcp.Client(server)`) or another transport. All robot calls
run on one thread that `RobotTools` owns.

## Writing a driver or a layer (SPI)

The sections above are the application API. This one is for code that extends
hiRobot: drivers and layers. These methods start with `_`; an application never
calls them, and a layer's `inner` does not expose them.

| Member | Kind | Description |
|---|---|---|
| `_send_action(action)` | abstract | Driver primitive. **Drivers implement this, never `send_action`** |
| `_estop()` | optional | Driver primitive behind `estop()`. Default: `damp()` if the driver has one |
| `_idle_step()` | optional, SPI | Let simulated time pass one control period under the last command. An assembly calls it on simulated parts the action did not mention. No-op by default; the toy sim and MuJoCo step |
| `_set_safety(gate)` | concrete, SPI | Attach or clear (`None`) the `SafetyGate`. For `make_robot` and tests; an application sets limits in the config's `safety:` section |
| `_run_onboard_skill(spec, args)` | optional | Driver primitive behind `run_onboard_skill`; `args` are already checked. Returns the expected run time in seconds for these arguments, or `None` for `spec.duration_s` |
| `_set_fault(reason)` / `_clear_fault()` | concrete, SPI | Put the robot in its fault state (a driver or layer that damped or stopped itself), so commands raise `RobotFaultError` until `reset()`; clear it |
| `_stop_token()`, `_commanding(token)`, `_wait(seconds, token)` | concrete, SPI | Let a long driver loop end at once when `estop()` is called from another thread (see [Writing a driver](#writing-a-driver)) |

### Writing a driver

Subclass `Robot`, implement `embodiment`, `connect`, `reset`,
`get_observation` and `_send_action`, and register it. The layout, manifest and
contract tests a new driver needs are in the README's
[Add a new robot](../README.md#add-a-new-robot). The first constructor
argument must be the `Embodiment`; the remaining keyword arguments are what users
put under `body:`. Set `simulated = True` on simulators. To offer robot
skills, override `has_onboard_skills`, `list_onboard_skills`, `_run_onboard_skill` and
`stop_onboard_skill`. A driver whose robot has a safe state overrides
`_estop()` (or provides `damp()`). A loop that runs for a while (a ramp, a wait
for the first state) takes `token = self._stop_token()` first, sends each step
inside `with self._commanding(token):` and sleeps with `self._wait(seconds, token)`,
so `estop()` from another thread ends it at once. Import vendor SDKs
inside `connect()` so the package imports without them, and test against a fake
SDK (`tests/drivers/contract.py`).

### Writing a layer

A layer is a package of its own that depends on `hirobot`. The steps:

1. **Subclass `Layer`.** The constructor is `cls(inner, **config)`: `inner` is
   the robot below, the rest is the layer's `layers:` entry without `type`.
2. **Declare what it takes.** Override `action_modes`, `direct_joints` and the
   other declarations only if they differ from `inner`'s. Toward `inner`, use
   only the modes `inner` declares.
3. **Implement it.** A passive layer implements `_send_action`: compute, then
   `self.inner.send_action(...)`. A layer with its own thread (whole-body
   control) only updates a target in `_send_action`, overrides `hold()` and
   `_estop()` (stop the thread only), and calls `self._set_fault(reason)` when
   the thread fails. Skills the layer runs go through [layer skills](#layer-hooks).
4. **Use only the public API of `inner`.** No `_`-prefixed names, no driver
   objects reached through `inner`.
5. **Register it** with `@register_layer("name")` in a module listed under the
   `hirobot.drivers` entry point in `pyproject.toml`:

   ```toml
   [project.entry-points."hirobot.drivers"]
   mypkg = "mypkg.plugin"
   ```

6. **Test it** on `toy` and the `*_mujoco` configs, then check its use of
   `inner` with `hirobot.testing.assert_layer_uses_public_api(MyLayer, config)`.
   On hardware: read-only first, then simulation started from the robot's pose,
   then the gantry with low gains.

A minimal passive layer, run on the toy UR5e (this block runs in the tests):

```python
import numpy as np

from hirobot import Action, ControlMode, Layer, Robot, make_robot, register_layer
from hirobot.loading.config import load_config


@register_layer("rate_limit")
class RateLimit(Layer):  # passive: limit how far each joint target moves per command

    def __init__(self, inner: Robot, max_step: float = 0.05):
        super().__init__(inner)
        self.max_step = float(max_step)
        self._last = None

    def reset(self, seed=None):
        self.inner.reset(seed)
        self._last = None

    def _send_action(self, action: Action) -> None:
        if action.mode is ControlMode.POSITION and action.joint_target is not None:
            if self._last is None:
                self._last = self.inner.get_observation().joint_pos
            step = np.clip(action.joint_target - self._last, -self.max_step, self.max_step)
            self._last = self._last + step
            action = action.model_copy(update={"joint_target": self._last})
        self.inner.send_action(action)


config = load_config("ur_ur5e")  # the toy UR5e
config["layers"] = [{"type": "rate_limit", "max_step": 0.05}]
robot = make_robot(config)
robot.connect()
robot.reset()
target = Action(mode=ControlMode.POSITION, joint_target=np.full(6, 0.3))
robot.send_action(target)
print(robot.get_observation().joint_pos)  # every joint moved 0.05 rad, not 0.3
```

### Layer hooks

`estop()` on any layer first e-stops the robot at the bottom of the stack
directly, then runs each layer's own `_estop()`, outermost first. Each gets
`ESTOP_LAYER_TIMEOUT_S` (0.2 s); one that raises or hangs is logged and
skipped, so a stuck layer cannot hold the stop back. Override `_estop` to stop
your layer only; do not call `inner.estop()`.

A layer is trusted code, reviewed like a kernel module. `self.inner` is a proxy
with the public API only: `_`-prefixed names (`_send_action`, `_set_safety`,
...) and an assembly's `body` / `end_effectors` raise `AttributeError`.
`hirobot.testing.assert_layer_uses_public_api(MyLayer, config)` runs the layer
on a toy robot and fails if it tried any of them.

Layer skills ([users' side](#layers)): a layer implements
`_layer_skills() -> List[RobotSkillSpec]`, `_start_layer_skill(spec, args)`
(start and return; inside it the layer's own `hold()` etc. pass),
`_stop_layer_skill()`, and calls `_layer_skill_finished()` when a skill ends on
its own. Its own thread commands `inner` as usual: that is below the arbiter.

## Extension points: cameras, end-effectors and audio

Cameras, end-effectors (dexterous hands, grippers) and audio devices have their own registries, separate
from the body driver. Like body drivers, they import vendor SDKs lazily.

### Camera drivers (`hirobot.devices.camera`, drivers in `hirobot.drivers.camera`)

| Name | Description |
|---|---|
| `register_camera(name)`, `get_camera(name)`, `available_cameras()` | Registry, same behavior as the driver registry |
| `CameraDriver(name)` | Abstract: `start()`, `latest() -> Optional[Frame]` (`(image, Stamp)`, `Stamp.t` on `time.monotonic()`, `seq` one more per new frame; must not block; `None` before the first frame), `stop()` (idempotent). RGB-D drivers also override `latest_rgbd() -> Optional[RgbdFrame]`; the default returns `latest()` with no depth. Optional `intrinsics() -> Optional[CameraIntrinsics]`: the intrinsics of the delivered images, read from the device after `start()`; default `None` |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | Runs a blocking `_grab() -> HxWx3 uint8 RGB` on a background thread and keeps only the newest frame. Subclasses implement `_grab` and optionally `_open` / `_close`. Exposes `wait_for_frame(timeout)`, `error`, `frames`, `errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`: the image and when it arrived (`Stamp(t, seq)`, `t` on `time.monotonic()`) |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`: colour image, depth registered to it (HxW float32, metres) or `None`, and their common `Stamp` |
| `intel_realsense` → `RealSenseCamera(name, serial=None, width=640, height=480, fps=30, depth=False, depth_width=None, depth_height=None, timeout_ms=1000, poll_hz=60.0)` | One Intel RealSense (D435i, D405, L515, ...) on the local USB of the computer running hiRobot, over `pyrealsense2`. `serial` picks a device when several are attached (else the first enumerated). With `depth: true` it is an RGB-D camera: the depth sensor is aligned to colour so `Observation.depth` shares the colour pixel grid, device units scaled to metres by the sensor's `depth_scale`. `depth_width` / `depth_height` set the depth stream size when it differs from colour (default: the colour size); the aligned depth image still comes out at the colour size. Needs `pyrealsense2` (`pip install pyrealsense2`); the L515 needs an older release (2.54.2 verified; 2.59.1 does not list it). `intrinsics()` is the colour stream's `get_intrinsics()` with its distortion model and coefficients |
| `orbbec_pyorbbecsdk` → `OrbbecCamera(name, serial=None, uid=None, width=640, height=480, fps=30, color_format="MJPG", depth=False, align="hw", timeout_ms=1000, poll_hz=60.0)` | One Orbbec camera (Dabai DC1, ...) on the local USB of the computer running hiRobot, over the 1.x `pyorbbecsdk` bindings (OrbbecSDK 1.10; older cameras do not enumerate under 2.x). `uid` (USB port path) or `serial` picks the device. With `depth: true` it is an RGB-D camera: depth aligned to colour (`align`: `hw` on the camera, `sw` in the SDK), device units scaled to metres. MJPEG / YUYV colour is decoded with OpenCV. The bindings are built from source; see [agilex_piper.md](setup/agilex_piper.md#6-orbbec-camera). `intrinsics()` is the colour intrinsics from `get_camera_param()`, Brown-Conrady distortion when set (a rational model, k4-k6, is reported without distortion, with a warning) |
| `stereolabs_zed` → `ZedCamera(name, serial=None, resolution="HD720", fps=30, depth=False, depth_mode="NEURAL", min_depth_m=0.1, poll_hz=60.0)` | One Stereolabs ZED (ZED 2i, ZED Mini, ...) on the local USB of the computer running hiRobot, over `pyzed`. `serial` picks a camera when several are attached (else the first). The image is the left camera's colour (`resolution`: `HD2K`, `HD1080`, `HD1200`, `HD720`, `SVGA`, `VGA`). With `depth: true` it is an RGB-D camera: depth in metres registered to the left image from the SDK's `depth_mode` (`NEURAL`, `ULTRA`, `QUALITY`, `PERFORMANCE`), 0 where there is no reading. Needs the ZED SDK, its `pyzed` wheel and an NVIDIA GPU. `intrinsics()` is the rectified left camera's calibration, no distortion |
| `agibot_cosine` → `AgibotCosineCamera(name, topic, depth_topic=None, depth_scale=0.001, max_pair_skew_s=0.005, pair_wait_s=0.02, queue=4, poll_hz=60.0)` | AgiBot G1 cameras on the robot's `cosine_bus` topics (`/camera/head_color` + `/camera/head_depth`, `/camera/hand_left_color`, ...). With `depth_topic` it is an RGB-D camera: each colour frame is paired with the depth frame of the same capture (`source_timestamp`), Z16 scaled to metres. Runs on the robot's computer with the vendor environment sourced (`LOCATOR_IP`, `AORTA_DISCOVERY_URI`) |
| `unitree_videohub` → `UnitreeVideohubCamera(name, network_interface=None, domain_id=0, timeout_s=1.0, poll_hz=30.0)` | G1 head camera |
| `unitree_image_server` → `UnitreeImageServerCamera(name, address, header=False, crop=None, timeout_s=1.0, poll_hz=60.0)` | H1 head camera: JPEG frames over ZeroMQ SUB from Unitree's teleoperation `image_server` (or `scripts/camera_image_server.py`). `header` strips the `Unit_Test` timestamp / frame id (kept in `remote_stamp` / `remote_frame_id`); `crop: [x, y, w, h]` takes one camera out of a side-by-side frame. Needs the `camera` extra |
| `video_file` → `VideoFileCamera(name, path, fps=5.0, loop=True)` | A video file played on its own timeline, sampled at `fps`. Without `loop` the last frame stays once the file ends (`ended` turns `True`). Needs the `video` extra (PyAV). Simulated |
| `array` → `ArrayCamera(name, frames=None, fps=10.0, advance_on_read=False)` | Cycles through frames in memory on the host clock; with no `frames`, one grey 64x64 frame. `advance_on_read=True` makes each `latest()` return the next frame (for tests). Simulated |
| `build_cameras(cameras, network_interface=None, sim_only=False) -> Dict[str, CameraDriver]` | One camera driver per camera with a `driver:`; `sim_only` builds only simulated ones. `make_robot` uses it for the embodiment's cameras |

A camera driver class sets `simulated = True` when it needs no hardware. Only
such drivers are built for a simulated body, and they do not get the body's
`network_interface`.

### End-effector drivers (`hirobot.devices.end_effector`, drivers in `hirobot.drivers.end_effector`)

Dexterous hands and grippers that have a driver of their own share one base
class and one registry. They differ only in their **interface**, which is how
the end-effector shows up in `Observation` / `Action`. The config picks it:

| Interface | Config | Observation | Command |
|---|---|---|---|
| `joints` | `joints: [...]` listed (they are in `Embodiment.joints`) | those joints in `joint_pos` / `joint_vel` / `joint_torque` | their slice of `joint_target` / `kp` / `kd`; in a non-joint mode, by `joint_names` |
| `opening` | no `joints`, `has_gripper: true` | `gripper[<name>]` in [0, 1], 0 closed, 1 open | `Action.gripper[<name>]`, sent in **every** mode |

Choose by what the device takes, not by its shape: a multi-finger hand is
always `joints`; a one-motor gripper that takes motor-level targets and gains
can be `joints` too, and one that takes only an opening target (it closes the
loop itself) is `opening`. A driver class lists the interfaces it supports in
`interfaces`; a config that asks for another is refused when the robot is built.

| Name | Description |
|---|---|
| `register_end_effector(name)`, `get_end_effector(name)`, `available_end_effectors()` | Registry |
| `EndEffectorDriver(name, joints, read_only=False)` | Abstract: `connect()`, `get_state()`, `damp()`. `joints` interface: `command(q, kp=None, kd=None)`, optional `set_gains(kp, kd)`. `opening` interface: `command_opening(opening)`, returns without waiting for the motion, a new target replaces the old one, `ValueError` outside [0, 1]. Optional: `reset()`, `disconnect()` (damps by default). `estop()`: ends a running `reset` ramp (`RobotStoppedError`), then damps. Attributes: `interfaces` (class, default `("joints",)`), `interface` (`"joints"` when joints are listed, else `"opening"`). Helpers: `joint_names`, `num_joints`, `clip(q)` (clips to `Joint.position_limit`) |
| `JointsState` | `joints` reading, a dataclass: `q`, `dq`, `tau` (NaN where untrusted), `stamp` (a `Stamp`, `time.monotonic()`; goes to `Observation.stamps[<name>]`), `extra` |
| `OpeningState` | `opening` reading, a dataclass: `opening` in [0, 1] (goes to `Observation.gripper[<name>]`), `stamp`, `extra` (contact, current, ...; merged into `Observation.extra` as `<name>_<key>`) |
| `check_opening(value)` | `value` as a float, `ValueError` if not finite or outside [0, 1] |
| `end_effector_joint_names(emb)`, `has_end_effector_drivers(emb)`, `body_embodiment(emb)`, `build_end_effectors(emb, network_interface=None, read_only=False)` | Helpers `make_robot` uses to split end-effectors from the body. `body_embodiment` also clears `has_gripper` on an `opening` end-effector, since its own driver drives that gripper |
| `unitree_handcmd` → `UnitreeDex3Hand(name, joints, side, network_interface=None, domain_id=0, default_kp=1.5, default_kd=0.2, ramp_seconds=1.0, ramp_hz=100.0, stale_s=0.5, read_only=False)` | Unitree Dex3-1 hand (`unitree_hg` `HandCmd_`); `side` is `"left"` or `"right"` |
| `unitree_motorcmds` → `UnitreeMotorCmdsHand(name, joints, cmd_topic, state_topic, motor_offset=0, num_motors=None, service=..., network_interface=None, domain_id=0, connect_timeout_s=2.0, stale_s=0.5, read_only=False)` | Any hand behind a Unitree serial-to-DDS service (`unitree_go` `MotorCmds_` / `MotorStates_`): Inspire on the H1 via `h1_inspire_service`, and the same layout for Dex1-1, BrainCo, Linker. Hands sharing one topic pair (Inspire 1.0.1: 12 motors, `motor_offset` 0 / 6) share one bus; slots no hand has commanded hold their measured pose |
| `inspire_sdkpy` → `InspireSdkpyHand(name, joints, side, network_interface=None, domain_id=0, cmd_topic=None, state_topic=None, connect_timeout_s=2.0, stale_s=0.5, read_only=False)` | Inspire RH56DFX through the inspire_sdkpy DDS bridge (`rt/inspire_hand/{ctrl,state}/{l,r}`, angles 0-1000 scaled to 0-1) |
| `robotiq_urcap` → `RobotiqUrcapGripper(name, joints, host, port=63352, speed=255, force=255, activate=False, connect_timeout_s=3.0, stale_s=0.5, poll_hz=20.0, read_only=False)` | Robotiq 2F gripper (`opening` interface) through the Robotiq URCap socket on a UR controller (TCP `port` at the controller IP). Registers are 0 open / 255 closed; the driver maps them to `opening` 1 open / 0 closed. `speed` / `force` are 0-255; `activate` activates on `connect()` if needed (a full close/open). `OpeningState.extra` has `object` (Robotiq object detection) and `fault`. `damp()` sends `GTO 0` (halt and hold). Python standard library only |

An end-effector driver is constructed as `cls(ee.name, joints, **spec)`, where
`joints` is empty for the `opening` interface and `spec` is the end-effector's
`driver:` dict without `type`. `make_robot` fills in the body's
`network_interface` when the spec sets none.

### Audio devices (`hirobot.devices.audio`, drivers in `hirobot.drivers.audio`)

Microphones and speakers are declared under `embodiment.microphones` /
`embodiment.speakers`. `make_robot` builds each one with a `driver:` as
`cls(name, **spec)` and attaches it to the `RobotAssembly`. Hardware devices
get the body's `network_interface` filled in. With a simulated driver only
devices whose class sets `simulated = True` are built; the rest are skipped. Audio does not go into `Observation`.
All PCM is mono int16 little-endian.

| Name | Description |
|---|---|
| `register_audio(name)`, `get_audio(name)`, `available_audio()` | Registry shared by microphones and speakers |
| `MicrophoneDriver(name)` | Abstract: `open()`, `read(timeout=None) -> Optional[AudioChunk]` (oldest first; `None` on timeout or once closed), `close()` (idempotent), `sample_rate` |
| `QueuedMicrophone(name, queue_s=10.0, retry_s=0.2)` | Runs a blocking `_receive() -> Optional[bytes]` on a background thread and queues up to `queue_s` of audio, dropping the oldest chunks beyond that. Subclasses implement `_receive` and optionally `_open` / `_close`. Exposes `wait_for_audio(timeout)`, `error`, `chunks`, `dropped` |
| `SpeakerDriver(name)` | Abstract: `open()`, `play(pcm, sample_rate=None) -> float` (blocks; returns the seconds played, less when stopped), `stop()` (callable from another thread), `close()`, `sample_rate`. Optional volume: `supports_volume` (class attribute, default `False`), `volume -> Optional[int]` (0-100, `None` when unknown), `set_volume(level: int)` (0-100, callable while `play()` runs; `ValueError` outside 0-100, `NotImplementedError` when `supports_volume` is `False`). Other optional device capabilities follow the same pattern: a `supports_<name>` flag the caller checks first |
| `SoftwareVolume` | Mixin for speakers without hardware volume: `supports_volume = True`, the level is kept in the driver and the driver scales its PCM with `apply_gain(pcm, level)` |
| `AudioChunk` | Dataclass: `pcm`, `sample_rate`, `stamp` (host receive time), `duration` |
| `resample(pcm, src_rate, dst_rate)` | Linear-interpolation resampling for speech |
| `unitree_voice_mic` → `UnitreeG1Microphone(name, network_interface=None, local_ip=None, group="239.168.123.161", port=5555, queue_s=10.0)` | G1 microphone: the audio board's UDP multicast, 16 kHz |
| `unitree_voice_speaker` → `UnitreeG1Speaker(name, network_interface=None, domain_id=0, volume=None, app_name="hirobot", chunk_s=1.0, lead_s=1.0, timeout_s=2.0)` | G1 speaker: the SDK `voice` service, 16 kHz. Hardware volume (`supports_volume`): `volume` reads the robot's current level (`GetVolume`), `set_volume(level)` sets it now (`RuntimeError` when not open or the call fails). The config `volume` is set on `open()`; if either changed it, the level the robot had on `open()` comes back on `close()` |
| `wav_mic` → `WavMicrophone(name, path, chunk_ms=20, realtime=True, tail_silence=1.0)` | Plays a 16-bit WAV file into the queue (at speaking speed with `realtime`, all at once without), adds `tail_silence` seconds of silence, then `read()` returns `None`. Simulated |
| `wav_speaker` → `WavSpeaker(name, path, realtime=True, sample_rate=16000)` | Records everything played into one WAV file, written on `close()`. With `realtime`, `play` takes as long as the audio and `stop()` cuts it. Volume is a software gain applied to what it records (starts at 100). Simulated |
| `null_speaker` → `NullSpeaker(name, realtime=True, sample_rate=16000)` | Plays nothing; with `realtime` it takes as long as the audio would. Has software volume (keeps the level only). Simulated |
| `sounddevice_mic` → `SoundDeviceMicrophone(name, sample_rate=16000, chunk_ms=20, device=None)` | The computer's input device. Needs the `audio` extra. Simulated (usable next to a simulator) |
| `sounddevice_speaker` → `SoundDeviceSpeaker(name, device=None, block_ms=50)` | The computer's output device, written in `block_ms` blocks so `stop()` is prompt. Volume is a software gain per block (starts at 100; the system mixer is not touched). Needs the `audio` extra. Simulated |
| `build_audio(microphones, speakers, network_interface=None, sim_only=False) -> (mics, speakers)` | Drivers for the devices with a `driver:`, same rules as `build_cameras` |
| `read_wav(path) -> (pcm, rate)`, `wav_bytes(pcm, rate)` | 16-bit WAV helpers in `hirobot.drivers.audio` (mono; the first channel of a multi-channel file) |
