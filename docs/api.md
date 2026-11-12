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

- [Top-level package](#top-level-package)
- [Quick example](#quick-example)
- [Embodiment](#embodiment)
- [Observation, Action, ActionChunk, ControlMode](#observation-action-actionchunk-controlmode)
- [Robot](#robot)
- [Command arbiter](#command-arbiter)
- [Robot skills](#robot-skills)
- [Building robots: make_robot and the driver registry](#building-robots-make_robot-and-the-driver-registry)
- [SafetyGate](#safetygate)
- [DofMap](#dofmap)
- [Kinematics](#kinematics)
- [Control loops](#control-loops)
- [RobotAssembly](#robotassembly)
- [Writing a driver or a layer (SPI)](#writing-a-driver-or-a-layer-spi)
- [Extension points: cameras, end-effectors and audio](#extension-points-cameras-end-effectors-and-audio)
- [Config file reference](#config-file-reference)
- [CLI](#cli)

## Top-level package

| Name | Kind | Module | Purpose |
|---|---|---|---|
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `AudioDevice`, `EmbodimentKind` | pydantic models / enum | `hirobot.core.embodiment` | Declarative robot I/O contract |
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
| `require_sdk` | function | `hirobot.loading.sdk` | Check a driver's vendor SDK against its `manifest.yaml` |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic models / exception | `hirobot.core.robot_skills` | Describe, start and refuse robot skills (walk, gestures) |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | class / dataclass / exception | `hirobot.control.safety` | Check every action before it reaches hardware |
| `DofMap` | class | `hirobot.core.dof` | Reorder joint vectors by name |
| `pose_compose`, `pose_inverse`, `pose_from_matrix`, `pose_from_axis_angle`, ... | functions | `hirobot.geometry.transforms` | Pose math on 7-vector poses `[x, y, z, qx, qy, qz, qw]` (not re-exported from `hirobot`) |
| `Kinematics`, `BodyPoses` | classes | `hirobot.geometry.kinematics` | Forward kinematics of an MJCF model (see [Kinematics](#kinematics); not re-exported from `hirobot`) |
| `__version__` | str | `hirobot` | Package version |

The interfaces and registries for cameras, end-effectors and audio are in `hirobot.devices.camera`,
`hirobot.devices.end_effector` and `hirobot.devices.audio`; the drivers themselves live under `hirobot.drivers`
(`drivers/<part>/<SDK vendor>/<SDK or protocol>/`). Both are covered in [Building robots](#building-robots-make_robot-and-the-driver-registry)
and [Extension points](#extension-points-cameras-end-effectors-and-audio).

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
| `EndEffector` | `name` | `str` | Key in `ee_poses` / `ee_targets` / `gripper` |
| | `has_gripper` | `bool` | Has a gripper commanded through `Action.gripper`. Required for an end-effector driver without `joints` (`opening` interface) and not allowed with `joints` |
| | `joints` | `List[str]` | Embodiment joints this end-effector actuates (finger joints), in its driver's motor order |
| | `driver` | `dict` or `None` | End-effector driver spec: `type` names a registered end-effector driver, other keys are its arguments. Needs `joints` or `has_gripper: true` |
| | `frame` | `EndEffectorFrame` or `None` | Link the pose is read from: `body` (link name), `pos` (m, in that link), `quat` `[x, y, z, w]` in that link. The `mujoco` body fills `ee_poses` for end-effectors with a frame; drivers whose SDK reports the pose (UR, Franka) ignore it; `Kinematics` computes the same pose from joint angles |
| `Camera` | `name` | `str` | Key in `Observation.images` |
| | `width`, `height` | `int` | Frame size as delivered |
| | `driver` | `dict` or `None` | Camera-driver spec: `type` names a registered camera driver, other keys are its arguments. Omit for cameras the body driver renders itself (simulators). The old key `source` is an error |
| | `depth` | `bool` | RGB-D camera: its driver also delivers a depth image registered to the colour image, in `Observation.depth` under the same name. Default `False` |
| | `fps` | `float` or `None` | Nominal frame rate. `stale_sources` uses it; `make_robot` warns when `control_hz` is more than twice it |
| | `mount` | `CameraMount` or `None` | Where the camera sits: `body` (link name), `pos` (m, in that link), `quat` `[x, y, z, w]` of the optical frame (z forward, x right, y down) in that link, `fovy` (vertical field of view, degrees). The `mujoco` body renders mounted cameras into `images`; real camera drivers ignore it |
| `AudioDevice` | `name` | `str` | Key in `robot.microphones` / `robot.speakers` |
| | `driver` | `dict` or `None` | Audio-driver spec: `type` names a registered microphone or speaker driver, other keys are its arguments |

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

## Robot

`hirobot.core.robot.Robot`, the abstract driver contract every robot, simulator and
wrapper implements.

**Threads.** `get_observation`, `get_status`, `now` and `estop` may be called
from any thread at any time. Every other method (`send_action`, `hold`,
`reset`, `run_robot_skill`, `set_gains`, `connect`, `disconnect`, ...) is
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
| `estop()` | concrete | Stop now; safe from any thread at any time, skips the safety gate and the [command arbiter](#command-arbiter). A long call on another thread (a `reset` ramp, a wait in `connect`) raises `RobotStoppedError` at its next step; the robot enters the fault state; a command already being sent finishes (at most `ESTOP_LOCK_TIMEOUT_S`, 0.2 s) and none goes out after; then the driver's own stop runs. Unitree and MuJoCo damp (Unitree with `builtin_skills` stops the skill), AgiBot G1 sends `MotionStop`, UR decelerates (`servoStop` / `speedStop` / `stopJ`), Franka stops its position controller; drivers without `damp()` (`toy`, `fourier_grx`, `agibot_aimrt`) only refuse further commands. `RobotAssembly` e-stops its body and every end-effector, or every part. On a layer, the robot at the bottom of the stack is e-stopped first, then each layer stops its own work, outermost first (see [Layer hooks](#layer-hooks); [Layers](#layers)) |
| `command_timeout_s` | attribute | Seconds without a new `VELOCITY` / `BASE_VELOCITY` command before the driver stops the motion and logs a warning (`hirobot.control.watchdog.CommandWatchdog`); not a fault, the next command runs as usual. Default 0.2 s on `toy`, `mujoco` (simulated time), `unitree_sdk2` (zero velocity, kp 0) and `ur_rtde` (`speedStop`); `agibot_gdk` has its own `command_timeout` (0.3 s) for the chassis; `None` on the other drivers. Set `body.command_timeout_s: null` to turn it off. Position targets are not watched |
| `action_modes` | class attribute | Modes the robot accepts. Callers that build actions read it |
| `motion_spaces` | class attribute | `MotionClip.space` values the robot tracks under `MOTION_CLIP`. Empty by default |
| `direct_joints` | class attribute | Joints a caller may command by name beside a non-joint mode. Empty by default |
| `cartesian_targets` | property | End-effectors that take `CARTESIAN` pose targets: every end-effector when `CARTESIAN` is in `action_modes`, else none. Overridden by robots that track only some |
| `now()` | optional | The robot's clock in seconds, what `Observation.stamps` times are on: `time.monotonic()` by default, simulated time in `toy` / `mujoco`. Wrappers return the inner robot's |
| `hold()` | optional | Keep the robot where it is, e.g. after a policy is stopped. By default sends the current `joint_pos` as a POSITION action when the driver accepts POSITION, otherwise does nothing. A simulator that takes only Cartesian deltas does nothing; Unitree with `builtin_skills` does nothing (the motion service keeps the robot standing). A driver with its own balance loop (a whole-body-controlled humanoid) must override it to command standing still (base velocity `[0, 0, 0]`): sending `joint_pos` freezes the joints and the humanoid falls. Goes through the safety gate |
| `set_gains(kp, kd)` | optional | Per-joint PD gains. No-op by default |
| `has_robot_skills` | property | Whether the driver offers robot skills at all. `False` by default |
| `list_robot_skills() -> List[RobotSkillSpec]` | optional | The skills the robot's own controller runs. Empty by default. See [Robot skills](#robot-skills) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | Check `args` against the skill's schema, then run it. Raises `RobotSkillError` |
| `stop_robot_skill()` | optional | Stop the running skill. No-op by default |
| `list_layer_skills() -> List[RobotSkillSpec]` | concrete | The skills the stack's layers run. Every robot has it; without layers it is empty. See [Layer skills](#layers) |
| `run_layer_skill(name, args=None) -> RobotSkillResult` | concrete | Start a layer skill. Without layers it raises `RobotSkillError` |
| `stop_layer_skill()` | concrete | Stop the running layer skill. No-op without layers |
| `layer_skill_running` | property | Whether a layer skill (of any layer in the stack) holds the command stream. `False` without layers |
| `disconnect()` | optional | Release and stop. No-op by default |
| `safety` | attribute | The attached gate, or `None` |
| `simulated` | class attribute | `True` for simulator drivers. With a simulated driver `make_robot` builds no end-effector drivers and only the cameras and audio devices whose class is also `simulated` (video file, WAV, the host's microphone) |
| `success`, `done`, `reward` | attributes | Episode outcome. Every driver in hiRobot (`toy`, `mujoco`, the real robots) leaves them `None`: they run no task. Task environments set them, e.g. hiBench's benchmark environments and its referee. A control loop can stop on them |

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
| Direct calls while held | `send_action`, `hold`, `reset`, `run_robot_skill` on the robot raise `RobotBusyError` (a `RuntimeError`). `estop`, `get_observation`, `get_status`, `now` are never checked |
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

## Robot skills

`hirobot.core.robot_skills`. A robot skill is a motion the robot's own controller
carries out (walk, sit, wave), started by name instead of by streaming joint
targets. Each driver declares the skills hiRobot supports on its robot, so
callers (an agent, a voice front end) list them and show them to a model
instead of hard-coding any.

```python
robot = make_robot("my_g1.yaml")      # robot.builtin_skills: g1
robot.connect()
[s.name for s in robot.list_robot_skills()]   # ['start_locomotion', 'walk', ...]
robot.run_robot_skill("start_locomotion")
robot.run_robot_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_robot_skill()
```

### `RobotSkillSpec`

| Field | Type | Description |
|---|---|---|
| `name` | `str` | Passed to `run_robot_skill` |
| `description` | `str` | One or two sentences for a model |
| `parameters` | `dict` | JSON Schema of the arguments (an `object` schema). Empty means none |
| `requires` | `List[str]` | Preconditions in plain words, e.g. `"robot standing on the ground"`. Informational |
| `duration_s` | `float` or `None` | Typical run time, if known |

### `run_robot_skill(name, args=None) -> RobotSkillResult`

Checks `args` against `parameters` before anything reaches the robot, then
runs it on the driver. Supported schema keywords: property
`type` (number / integer / boolean / string), `minimum`, `maximum`,
`exclusiveMinimum`, `enum`, `default`, and `required`. Missing arguments take
their `default`; unknown arguments are refused. It returns once the robot has
accepted the command, not when the motion ends. `RobotSkillResult` carries
`name`, the checked `args` (defaults filled in) and `duration_s`.

`RobotSkillError` is raised for an unknown skill, a bad argument, or a robot
that refused.

### Wrappers

`RobotAssembly` forwards all four skill calls to its body driver, and lists each
child part's skills under the part's prefix (`<part>/<skill>`).

### Drivers with skills

| Driver | Config | Skills |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 locomotion and arm gestures, see the [Unitree setup guide](setup/unitree.md#robot-skills-g1) |

Every other driver offers none.

## Building robots: make_robot and the driver registry

### `load_embodiment(config) -> Embodiment`

The `Embodiment` a robot config (`assembly:`) or a device file (`device:`)
describes: a built-in name (`"unitree_g1"`, the same as `"robots/unitree_g1"`),
a file path, or an already-loaded dict, after `defaults:` are expanded (see
[Config file reference](#config-file-reference)). For an assembly with parts the
embodiment is the merged tree, every part's names under its `<part>/` prefix.
Nothing is connected or built. Raises `ValueError` if the config is neither.

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
ships none; plugin packages (hiWBC) register them.

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
lifecycle, observations, `get_status`, `hold`, gains and robot skills; `fault`
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
behavior, a macro), apart from the vendor's robot skills:
`list_layer_skills()` (this layer's and the layers' below), `run_layer_skill(name, args=None) -> RobotSkillResult`
(arguments checked against the schema, returns once started),
`stop_layer_skill()` and `layer_skill_running` (true while a skill of this layer
or one below runs). The four are on every `Robot`, so callers need not check
whether the outermost robot is a layer. While a skill runs, the layer
holds the stack's [command lease](#command-arbiter) with `Priority.LAYER_SKILL`:
a caller's lease is preempted, and direct commands raise `RobotBusyError` until
the skill ends or `stop_layer_skill()` is called. A `TAKEOVER` preempts the
skill (it is stopped). `estop()` ends it, so the caller can `reset()`.

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
| `toy` | `ToyRobot` | `drivers/sim/toy/` | Toy kinematic integrator, no dependencies |
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo physics, a robot of its own; renders cameras that have a `mount`; `ee_poses` for end-effectors that have a `frame`; `set_state` starts it from a measured pose |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1 (精灵 G01): arms, waist, head and grippers over the GDK's DDS topics; chassis over its controller box |
| `agibot_aimrt` | `AgibotX1Robot` | `drivers/body/agibot/aimrt/` | AgiBot X1 |
| `franka_libfranka` | `FrankaRobot` | `drivers/body/franka/libfranka/` | Franka FR3 |
| `ur_rtde` | `URRobot` | `drivers/body/ur/rtde/` | Universal Robots UR5e |

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

kin = Kinematics.from_mjcf("~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")
poses = kin.forward(obs.joint_pos, base_pos=[0, 0, 0.79], base_quat=[0, 0, 0, 1],
                    joint_names=robot.embodiment.joint_names)
pos, quat = poses["torso_link"]
```

## Control loops

hiRobot has no policy classes, control loop or model-server client; it only
supplies the `Robot` a loop drives. What a loop uses from a driver:
`reset(seed=None)`, `get_observation()`, `send_action()`, `hold()` after a stop,
`embodiment.control_hz` for pacing a real robot, `simulated` (step a simulator
unpaced), and on simulators `success` / `done` / `reward`.
`ActionChunk.step(i)` turns one step of a predicted chunk into an `Action`.

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
| Skills | Part skills listed as `<part>/<skill>`; `run_robot_skill("g1/wave")` runs it on that part |
| Lifecycle | `connect` connects parts then the node's own cameras, undoing on failure. `reset`, `hold`, `damp`, `estop`, `disconnect`, `stop_robot_skill` reach every part even when one fails, then raise the first error. `reset` also clears the fault state |
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
| `_run_robot_skill(spec, args)` | optional | Driver primitive behind `run_robot_skill`; `args` are already checked |
| `_set_fault(reason)` / `_clear_fault()` | concrete, SPI | Put the robot in its fault state (a driver or layer that damped or stopped itself), so commands raise `RobotFaultError` until `reset()`; clear it |
| `_stop_token()`, `_commanding(token)`, `_wait(seconds, token)` | concrete, SPI | Let a long driver loop end at once when `estop()` is called from another thread (see [Writing a driver](#writing-a-driver)) |

### Writing a driver

Subclass `Robot`, implement `embodiment`, `connect`, `reset`,
`get_observation` and `_send_action`, and register it. The layout, manifest and
contract tests a new driver needs are in the README's
[Add a new robot](../README.md#add-a-new-robot). The first constructor
argument must be the `Embodiment`; the remaining keyword arguments are what users
put under `body:`. Set `simulated = True` on simulators. To offer robot
skills, override `has_robot_skills`, `list_robot_skills`, `_run_robot_skill` and
`stop_robot_skill`. A driver whose robot has a safe state overrides
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
| `CameraDriver(name)` | Abstract: `start()`, `latest() -> Optional[Frame]` (`(image, Stamp)`, `Stamp.t` on `time.monotonic()`, `seq` one more per new frame; must not block; `None` before the first frame), `stop()` (idempotent). RGB-D drivers also override `latest_rgbd() -> Optional[RgbdFrame]`; the default returns `latest()` with no depth |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | Runs a blocking `_grab() -> HxWx3 uint8 RGB` on a background thread and keeps only the newest frame. Subclasses implement `_grab` and optionally `_open` / `_close`. Exposes `wait_for_frame(timeout)`, `error`, `frames`, `errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`: the image and when it arrived (`Stamp(t, seq)`, `t` on `time.monotonic()`) |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`: colour image, depth registered to it (HxW float32, metres) or `None`, and their common `Stamp` |
| `intel_realsense` → `RealSenseCamera(name, serial=None, width=640, height=480, fps=30, depth=False, depth_width=None, depth_height=None, timeout_ms=1000, poll_hz=60.0)` | One Intel RealSense (D435i, D405, L515, ...) on the local USB of the computer running hiRobot, over `pyrealsense2`. `serial` picks a device when several are attached (else the first enumerated). With `depth: true` it is an RGB-D camera: the depth sensor is aligned to colour so `Observation.depth` shares the colour pixel grid, device units scaled to metres by the sensor's `depth_scale`. `depth_width` / `depth_height` set the depth stream size when it differs from colour (default: the colour size); the aligned depth image still comes out at the colour size. Needs `pyrealsense2` (`pip install pyrealsense2`); the L515 needs an older release (2.54.2 verified; 2.59.1 does not list it) |
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
| `SpeakerDriver(name)` | Abstract: `open()`, `play(pcm, sample_rate=None) -> float` (blocks; returns the seconds played, less when stopped), `stop()` (callable from another thread), `close()`, `sample_rate` |
| `AudioChunk` | Dataclass: `pcm`, `sample_rate`, `stamp` (host receive time), `duration` |
| `resample(pcm, src_rate, dst_rate)` | Linear-interpolation resampling for speech |
| `unitree_voice_mic` → `UnitreeG1Microphone(name, network_interface=None, local_ip=None, group="239.168.123.161", port=5555, queue_s=10.0)` | G1 microphone: the audio board's UDP multicast, 16 kHz |
| `unitree_voice_speaker` → `UnitreeG1Speaker(name, network_interface=None, domain_id=0, volume=None, app_name="hirobot", chunk_s=1.0, lead_s=1.0, timeout_s=2.0)` | G1 speaker: the SDK `voice` service, 16 kHz. `volume` is set on `open()` and restored on `close()`; `set_volume(v)` changes it now |
| `wav_mic` → `WavMicrophone(name, path, chunk_ms=20, realtime=True, tail_silence=1.0)` | Plays a 16-bit WAV file into the queue (at speaking speed with `realtime`, all at once without), adds `tail_silence` seconds of silence, then `read()` returns `None`. Simulated |
| `wav_speaker` → `WavSpeaker(name, path, realtime=True, sample_rate=16000)` | Records everything played into one WAV file, written on `close()`. With `realtime`, `play` takes as long as the audio and `stop()` cuts it. Simulated |
| `null_speaker` → `NullSpeaker(name, realtime=True, sample_rate=16000)` | Plays nothing; with `realtime` it takes as long as the audio would. Simulated |
| `sounddevice_mic` → `SoundDeviceMicrophone(name, sample_rate=16000, chunk_ms=20, device=None)` | The computer's input device. Needs the `audio` extra. Simulated (usable next to a simulator) |
| `sounddevice_speaker` → `SoundDeviceSpeaker(name, device=None, block_ms=50)` | The computer's output device, written in `block_ms` blocks so `stop()` is prompt. Needs the `audio` extra. Simulated |
| `build_audio(microphones, speakers, network_interface=None, sim_only=False) -> (mics, speakers)` | Drivers for the devices with a `driver:`, same rules as `build_cameras` |
| `read_wav(path) -> (pcm, rate)`, `wav_bytes(pcm, rate)` | 16-bit WAV helpers in `hirobot.drivers.audio` (mono; the first channel of a multi-channel file) |

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
model:                      # optional MJCF for sim / collision / kinematics
  mjcf: ~/models/ur5e.xml
driver: {type: ur_rtde}     # registered driver name + its constructor kwargs
```

A camera file carries `width` / `height` / `fps` / `depth` / `fovy`; an
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
| `hirobot info --config NAME_OR_PATH [--resolved]` | Print the embodiment: kind, control rate, floating base, joints, end-effectors, cameras. `--resolved` prints the whole config after `defaults:` are merged |
| `hirobot configs [GROUP]` | List the built-in configs (`robots`, `body`, `end_effector`, `camera`, `microphone`, `speaker`). Names under `robots/` also work without the prefix (`unitree_g1`) |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | Run a simulated robot: connect, hold the start pose for N s (default 3), print the joint error left, the base height, the rendered images and `get_observation` time. Says when a free base fell. Exit 2 for a real body; `--viewer` needs a MuJoCo body |
| `hirobot check --sim CONFIG [--against-real] [--driver PART:KEY=VALUE ...] [--tolerance 0.02]` | Build the world the simulation check uses, without stepping it: every body needs a `model:`, every joint must be in its MJCF with an actuator, every end-effector frame a body of it; joint limits that differ from the MJCF's range by more than 0.1 rad are printed as warnings. `--against-real` also opens every body's real driver `read_only: true` (refused without that mode), reads one observation and prints each end effector's measured vs model position at those joint angles. Exit 1 on an error or an error above `--tolerance` m. Library: `hirobot.sim.check_sim` |
| `hirobot view CONFIG [--model MUJOCO_CONFIG] [--body KEY=VALUE ...] [--hz 30] [--seconds N]` | Show a real robot's pose in a MuJoCo viewer. Opens CONFIG with `read_only: true` (a driver without a `read_only` mode is refused), copies the measured joints (by name) and base orientation into the model each frame, and never sends a command. `--model` defaults to the built-in `<config>_mujoco`. Exit 2 for a simulated robot or no model. Library: `hirobot.sim.view` |
