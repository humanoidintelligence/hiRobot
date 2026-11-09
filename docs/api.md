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
- [Robot skills](#robot-skills)
- [Building robots: make_robot and the driver registry](#building-robots-make_robot-and-the-driver-registry)
- [SafetyGate](#safetygate)
- [DofMap](#dofmap)
- [Kinematics](#kinematics)
- [Control loops](#control-loops)
- [RobotAssembly and TwinRobot](#robotassembly-and-twinrobot)
- [Extension points: cameras, end-effectors and audio](#extension-points-cameras-end-effectors-and-audio)
- [Config file reference](#config-file-reference)
- [CLI](#cli)

## Top-level package

| Name | Kind | Module | Purpose |
|---|---|---|---|
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `AudioDevice`, `EmbodimentKind` | pydantic models / enum | `hirobot.embodiment` | Declarative robot I/O contract |
| `Observation`, `Action`, `ActionChunk`, `ControlMode`, `MotionClip`, `JOINT_MODES`, `MOTION_SPACES` | pydantic models / enum / tuples | `hirobot.types` | Typed I/O between robot and policy |
| `Robot` | abstract class | `hirobot.robot` | Driver contract |
| `RobotAssembly` | `Robot` | `hirobot.assembly` | Body driver plus attached cameras, end-effectors (hands, grippers), microphones and speakers |
| `RobotGroup` | `Robot` | `hirobot.group` | Several complete robots plus scene cameras driven as one (see [RobotGroup](#robotgroup)) |
| `RobotFaultError` | exception | `hirobot.robot` | Raised by `send_action` while the robot is in a fault state |
| `RobotStoppedError` | exception | `hirobot.robot` | `RobotFaultError` subclass: a long call (`connect`, `reset`) gave up because `estop()` ran |
| `register_body`, `get_body`, `available_bodies` | functions | `hirobot.registry` | Name-based driver registry |
| `register_layer`, `get_layer`, `available_layers` | functions | `hirobot.registry` | Registry of layers stacked by `layers:` (see [Layers](#layers)) |
| `load_embodiment`, `make_robot` | functions | `hirobot` | Build an embodiment / robot from a config name, file or dict |
| `require_sdk` | function | `hirobot.sdk` | Check a driver's vendor SDK against its `manifest.yaml` |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic models / exception | `hirobot.robot_skills` | Describe, start and refuse robot skills (walk, gestures) |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | class / dataclass / exception | `hirobot.safety` | Check every action before it reaches hardware |
| `DofMap` | class | `hirobot.dof` | Reorder joint vectors by name |
| `pose_compose`, `pose_inverse`, `pose_from_matrix`, `pose_from_axis_angle`, ... | functions | `hirobot.transforms` | Pose math on 7-vector poses `[x, y, z, qx, qy, qz, qw]` (not re-exported from `hirobot`) |
| `Kinematics`, `BodyPoses` | classes | `hirobot.kinematics` | Forward kinematics of an MJCF model (see [Kinematics](#kinematics); not re-exported from `hirobot`) |
| `__version__` | str | `hirobot` | Package version |

The interfaces and registries for cameras, end-effectors and audio are in `hirobot.camera`,
`hirobot.end_effector` and `hirobot.audio`; the drivers themselves live under `hirobot.drivers`
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

`hirobot.embodiment`. Pydantic models describing what a robot looks like to a
policy, independent of the driver. Usually loaded from the `embodiment:` section
of a config with `load_embodiment(path)`.

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

`hirobot.types`. Pydantic models that accept numpy arrays. Every optional field
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
| `stamps` | `Dict[str, Stamp]` | every driver sets `stamps["body"]`; hands, grippers with their own topic and cameras add theirs under their name (`left/body` etc. in a group). `Stamp(t, seq)`: `t` is when the reading arrived on `robot.now()`, `seq` counts the source's readings from 1. Age of a source: `robot.now() - obs.stamps[name].t`; a camera frame seen again has the same `seq`. `hirobot.stamps.stale_sources(obs, robot.now(), max_age=None, embodiment=None)` lists sources older than `max_age` (seconds, or per source) or, by default, 3 nominal periods (`Camera.fps`, `control_hz`) |
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
| `members` | `Dict[str, Action]` | `{}` | Robot groups only: member name -> that member's action, in its own names and mode. Every other field but `extra` must then be empty; a single robot refuses it (`SafetyError`) |

Construction raises `ValueError` for `MOTION_CLIP` without `motion`, `BASE_POSE`
without `base_target`, `joint_names` with a joint mode, duplicates or no
`joint_target`, or `members` beside another command field.

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
| `members` | `Dict[str, ActionChunk]` | Robot groups: member name -> its chunk. All have the same horizon (`ValueError` otherwise), the other fields stay empty, and `step(i)` gives an `Action` with `members` |

- `horizon -> int` (also `len(chunk)`): `T`, taken from `joint_targets`,
  `base_velocity`, `base_target` or `latent`, then `ee_targets`, then `gripper`;
  `0` if all are empty.
- `step(i) -> Action`: the `i`-th step as a single `Action`. Raises `IndexError`
  outside `[0, horizon)`.

## Robot

`hirobot.robot.Robot`, the abstract driver contract every robot, simulator and
wrapper implements.

| Member | Kind | Description |
|---|---|---|
| `embodiment` | abstract property | The `Embodiment` this driver exposes |
| `connect()` | abstract | Open the connection. Called once before control |
| `reset(seed=None)` | abstract | Bring the robot to a safe initial state. Simulators use `seed` to pick the scene (e.g. an env seed or init-state index); other drivers ignore it |
| `get_observation() -> Observation` | abstract | Read sensors |
| `send_action(action)` | concrete | `check_action`, then `_send_action`. Raises `SafetyError` on refusal and nothing is sent, including when `action.joint_names` lists a joint outside `direct_joints`. Raises `RobotFaultError` in the fault state, and `RuntimeError` when the robot is a `RobotGroup` member and the call does not come through the group |
| `check_action(action) -> Action` | concrete | Every check `send_action` runs, without sending; returns the (possibly clamped) action. The group runs it on every member before sending to any |
| `fault` | property | Why the robot refuses commands, or `None`. Set by `set_fault(reason)` (a driver or layer that damped or stopped itself) and by `estop()`. Cleared when `reset()` returns (every subclass's `reset` is wrapped to call `clear_fault()`), unless `estop()` ran meanwhile. Wrappers (`RobotAssembly`, `TwinRobot`) report their inner robot's too. Also `get_status().fault` |
| `estop()` | concrete | Stop now; safe from any thread at any time, skips the safety gate and a group's owner check. A long call on another thread (a `reset` ramp, a wait in `connect`) raises `RobotStoppedError` at its next step; the robot enters the fault state; a command already being sent finishes (at most `ESTOP_LOCK_TIMEOUT_S`, 0.2 s) and none goes out after; then `_estop()` runs. Unitree and MuJoCo damp (Unitree with `builtin_skills` stops the skill), AgiBot G1 sends `MotionStop`, UR decelerates (`servoStop` / `speedStop` / `stopJ`), Franka stops its position controller; drivers without `damp()` (`toy`, `fourier_grx`, `agibot_aimrt`) only refuse further commands. `RobotAssembly`, `RobotGroup` and `TwinRobot` e-stop every part |
| `_estop()` | optional | Driver primitive behind `estop()`. Default: `damp()` if the driver has one |
| `idle_step()` | optional | Let simulated time pass one control period under the last command. A group calls it on simulated members the action did not mention. No-op by default; the toy sim and MuJoCo step |
| `action_modes` | class attribute | Modes `_send_action` accepts. Callers that build actions read it |
| `motion_spaces` | class attribute | `MotionClip.space` values the robot tracks under `MOTION_CLIP`. Empty by default |
| `direct_joints` | class attribute | Joints a caller may command by name beside a non-joint mode. Empty by default |
| `cartesian_targets` | property | End-effectors that take `CARTESIAN` pose targets: every end-effector when `CARTESIAN` is in `action_modes`, else none. Overridden by robots that track only some |
| `_send_action(action)` | abstract | Driver primitive. **Drivers implement this, never `send_action`** |
| `now()` | optional | The robot's clock in seconds, what `Observation.stamps` times are on: `time.monotonic()` by default, simulated time in `toy` / `mujoco`. Wrappers return the inner robot's |
| `hold()` | optional | Keep the robot where it is, e.g. after a policy is stopped. By default sends the current `joint_pos` as a POSITION action when the driver accepts POSITION, otherwise does nothing. A simulator that takes only Cartesian deltas does nothing; Unitree with `builtin_skills` does nothing (the motion service keeps the robot standing). A driver with its own balance loop (a whole-body-controlled humanoid) must override it to command standing still (base velocity `[0, 0, 0]`): sending `joint_pos` freezes the joints and the humanoid falls. Goes through the safety gate |
| `set_gains(kp, kd)` | optional | Per-joint PD gains. No-op by default |
| `set_safety(gate)` | concrete | Attach or clear (`None`) the `SafetyGate` |
| `has_robot_skills` | property | Whether the driver offers robot skills at all. `False` by default |
| `list_robot_skills() -> List[RobotSkillSpec]` | optional | The skills the robot's own controller runs. Empty by default. See [Robot skills](#robot-skills) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | Check `args` against the skill's schema, then `_run_robot_skill`. Raises `RobotSkillError` |
| `_run_robot_skill(spec, args)` | optional | Driver primitive behind `run_robot_skill`; `args` are already checked |
| `stop_robot_skill()` | optional | Stop the running skill. No-op by default |
| `disconnect()` | optional | Release and stop. No-op by default |
| `safety` | attribute | The attached gate, or `None` |
| `simulated` | class attribute | `True` for simulator drivers. With a simulated driver `make_robot` builds no end-effector drivers and only the cameras and audio devices whose class is also `simulated` (video file, WAV, the host's microphone) |
| `success`, `done`, `reward` | attributes | Episode outcome. Every driver in hiRobot (`toy`, `mujoco`, the real robots) leaves them `None`: they run no task. Task environments set them, e.g. hiBench's benchmark environments and its referee. A control loop can stop on them |

## Robot skills

`hirobot.robot_skills`. A robot skill is a motion the robot's own controller
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
calls the driver's `_run_robot_skill`. Supported schema keywords: property
`type` (number / integer / boolean / string), `minimum`, `maximum`,
`exclusiveMinimum`, `enum`, `default`, and `required`. Missing arguments take
their `default`; unknown arguments are refused. It returns once the robot has
accepted the command, not when the motion ends. `RobotSkillResult` carries
`name`, the checked `args` (defaults filled in) and `duration_s`.

`RobotSkillError` is raised for an unknown skill, a bad argument, or a robot
that refused.

### Wrappers

`RobotAssembly` forwards all four members to its body driver. `TwinRobot`
forwards them to the real robot in `mirror` mode and offers none in `shadow`
mode, since the twin cannot run the vendor's controller.

### Drivers with skills

| Driver | Config | Skills |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 locomotion and arm gestures, see the [Unitree setup guide](setup/unitree.md#robot-skills-g1) |

Every other driver offers none.

## Building robots: make_robot and the driver registry

### `load_embodiment(config) -> Embodiment`

Reads the `embodiment:` section of a config: a built-in name (`"unitree_g1"`, the
same as `"robots/unitree_g1"`), a file path, or an already-loaded dict, after
`defaults:` are expanded (see [Config file reference](#config-file-reference)). Raises `ValueError` if it is
missing.

### `make_robot(config, body_overrides=None, sim_overrides=None) -> Robot`

Builds the configured robot. `config` is a built-in name, a file path, or a dict.
Steps, in order:

1. Expand `defaults:`, load the embodiment and look up `body.driver` (default `toy`) in the registry.
2. With a real driver and end-effectors that have a `driver:`, build one
   end-effector driver for each and give the body driver only the body joints
   (and none of the grippers those drivers own).
3. Instantiate the driver as `driver_cls(embodiment, **robot_cfg)`. All other
   `body:` keys are passed through as constructor arguments.
4. If `sim.mode` is `mirror` or `shadow`, wrap the robot with a MuJoCo twin in a
   `TwinRobot`. `shadow` forces `read_only=True` on the real driver.
5. If `layers:` is set: join body and end-effectors in a `RobotAssembly` (so every layer
   sees every joint), attach a `SafetyGate` to it, and wrap it in each layer in
   order (see [Layers](#layers)).
6. With a real driver, build a camera driver for each camera with a `driver:`
   and an audio driver for each microphone / speaker with a `driver:`, and wrap
   everything in a `RobotAssembly`.
7. Attach a `SafetyGate` from the embodiment and the `safety:` section to the
   outermost robot, unless `safety: false`. With layers, the joint commands they
   send are checked by the gate from step 5, and what the caller sends by this one.

`body_overrides` is merged over `body:` and `sim_overrides` over `sim:`, so
host-specific values (network interface, IP, mode) can come from the command
line.

Raises `ValueError` for an invalid `sim.mode`, a `sim:` section without `model`,
`shadow` combined with `release_motion_mode`, a camera / end-effector / audio / layer spec
without `type`, an end-effector driver that does not support the interface its config asks for, a `layers:` that is not a list, or a speaker driver listed as a
microphone (or the reverse). Raises `KeyError` for an unknown driver or layer name.

### Layers

A layer is a `Robot` that wraps another robot and changes how it is commanded:
a whole-body controller that takes `BASE_VELOCITY` / `MOTION_CLIP` / `CARTESIAN`
and sends balanced joint targets to the robot underneath every tick. hiRobot
ships none; plugin packages (hiWBC) register them.

```python
from hirobot import Robot, register_layer

@register_layer("my_wbc")
class MyWbc(Robot):
    def __init__(self, inner: Robot, **config): ...
```

```yaml
layers:                     # first entry innermost
  - type: my_wbc            # registered layer name
    controller: sonic       # every other key -> the layer's constructor
```

A layer sends its joint commands through `inner.send_action`, so the gate
`make_robot` attaches underneath checks them. It reports what it accepts through
`action_modes`, `motion_spaces`, `cartesian_targets` and `direct_joints`.
With real dexterous hands the layer's `inner` is the body-plus-end-effectors
`RobotAssembly`, so it can drive the fingers itself, or leave them free
(`direct_joints`) for the caller to command by `Action.joint_names`. Register
layers from the same `hirobot.drivers` entry point as drivers.

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
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo physics, a robot of its own; renders cameras that have a `mount`; `ee_poses` for end-effectors that have a `frame`; also the real-robot twin |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1 (精灵 G01): arms, waist, head and grippers over the GDK's DDS topics; chassis over its controller box |
| `agibot_aimrt` | `AgibotX1Robot` | `drivers/body/agibot/aimrt/` | AgiBot X1 |
| `franka_libfranka` | `FrankaRobot` | `drivers/body/franka/libfranka/` | Franka FR3 |
| `ur_rtde` | `URRobot` | `drivers/body/ur/rtde/` | Universal Robots UR5e |

Constructor arguments of each hardware driver are documented in its
[setup guide](setup/README.md).

### SDK manifests (`hirobot.sdk`)

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

## SafetyGate

`hirobot.safety`. The check `Robot.send_action` runs on every action before it
reaches the driver.

```python
SafetyGate(num_dofs=None, joint_limit=None, clamp=False, gripper_range=None)
SafetyGate.from_embodiment(embodiment, **kwargs)   # num_dofs = embodiment.num_dofs
gate.check(action, embodiment=None) -> SafetyVerdict
```

| Argument | Effect |
|---|---|
| `num_dofs` | `joint_target` must have this length (unless `joint_names` is set: then one value per name, and every name must be a joint of `embodiment`). `None` skips the check |
| `joint_limit` | Symmetric bound `[-joint_limit, joint_limit]` (rad) on `joint_target`. `None` skips it |
| `clamp` | `False` (default): out-of-bound targets are refused. `True`: they are clipped into bounds |
| `gripper_range` | `[low, high]` for gripper values; values are clipped into it. `None` skips it |

Always checked: `joint_target` is 1-D, finite, and gripper values are finite;
`base_velocity`, `base_target`, `latent`, every `ee_targets` entry and every
array of `motion` are finite.

Not checked: per-joint bounds (`Joint.position_limit`; only end-effector drivers clip to
them), how far a target is from the measured joint position, and how far it is
from the previous target. On robots whose limits are not symmetric (AgiBot G1's
elbows, UR, Franka) the robot's own controller is the only per-joint limit, and
a bad first target can make a servoing arm jump. `joint_limit` applies to
`joint_target` in every joint mode, including `VELOCITY`, where it holds
velocities (rad/s).

`SafetyVerdict` has `allowed: bool`, `action` (possibly clamped, valid only when
allowed) and `reason: str`. `SafetyError` (a `RuntimeError`) is what
`send_action` raises with that reason.

## DofMap

`hirobot.dof.DofMap(source_names, target_names)` maps between two joint orderings
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

`hirobot.kinematics.Kinematics.from_mjcf(path, backend="numpy")` loads an MJCF model and
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
from hirobot.kinematics import Kinematics

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

## RobotAssembly and TwinRobot

Both are `Robot` wrappers that `make_robot` builds from the config. You rarely
construct them directly. Attributes they do not define (for example a driver's
`raw_state`) are forwarded to the wrapped driver.

### `RobotAssembly(body, cameras=None, end_effectors=None, embodiment=None, first_frame_timeout_s=3.0, microphones=None, speakers=None, first_audio_timeout_s=2.0)`

`hirobot.assembly`. A body driver plus camera drivers, end-effector drivers and audio devices.

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

### `TwinRobot(real, sim, mode="mirror", mirror_hz=50.0)`

`hirobot.twin`. A real driver paired with a `MujocoRobot` twin. Needs the
`sim` extra.

| Mode | Commands go to | Twin |
|---|---|---|
| `mirror` | Real robot | A background thread copies measured joint state into the sim and renders it (kinematic, no physics) |
| `shadow` | Simulation only | Seeded once from the real pose. The real driver must be read-only, checked on `connect()`. Offers no robot skills |

- `active`: the driver receiving commands (`sim` in shadow mode, else `real`).
- `mirror_error`: the last exception raised by the mirroring thread, or `None`.

## RobotGroup

`hirobot.group.RobotGroup(members, cameras=None, name="robot_group", control_hz=None, scene_cameras=(), first_frame_timeout_s=3.0, skew_warn_s=None, base_poses=None, expose=None)`.
Several complete robots (each a `make_robot` stack with its own hands, cameras,
sim twin, layers and gate) plus scene cameras, as one `Robot`. Built from a
config by `make_robot` when the config has a `group:` section, or directly:

```python
group = RobotGroup({"left": make_robot("franka_fr3"), "right": make_robot("ur_ur5e")},
                   cameras={"scene": scene_camera})
```

| Topic | Behavior |
|---|---|
| Names | Every member joint, end-effector, camera, microphone and speaker is `<member>/<name>` in `embodiment`. Scene cameras keep their own names. `expose={"ee": {"left/ee": "left_arm"}, "cameras": {"scene": "agentview"}}` renames end-effectors and cameras (not joints); the exposed name is then the only one, in the embodiment, the observation and actions. Unknown names or clashes after renaming raise `ValueError` |
| Frames | `base_poses={member: [x, y, z, qx, qy, qz, qw]}` places members in the world. `ee_poses` are then world-frame, and `ee_targets` are taken as world-frame and converted to each member's base frame before sending; `CARTESIAN_DELTA` translation and rotation are rotated by the base orientation. A member without a base pose keeps its own frame. For a floating-base member the base pose is the origin of its odometry: the group composes it with the member's current `base_position` / `base_orientation`. While such a member reports no `base_position`, its `ee_poses` go to `extra["<member>/ee_poses"]` (own frame), its end-effectors leave `cartesian_targets`, and world-frame targets for it are refused; a `members` action with `extra={"frame": "base"}` still gives it own-frame targets |
| `embodiment` | Joints concatenated in member order. `floating_base` when exactly one member has one; `control_hz` defaults to the slowest member's |
| `get_observation()` | Joint vectors concatenated; `ee_poses`, `gripper`, `images`, `depth`, `stamps`, `extra` keys prefixed (`stamps["left/body"]`); scene camera frames added under their own names; a warning when the members are read more than `skew_warn_s` apart (default half a period). `now()`: the host clock with any real member, else the first member's simulated time. One floating-base member: its `base_*` fields stay top-level; several: in `extra["<member>/base_*"]` |
| `send_action(action)` | Flat or per member. Per member (`Action.members`): each sub-action goes to its member as is, apart from the frame conversion; modes may differ. Flat: one mode for every member. `joint_target` / `kp` / `kd` split by member joint counts, or by name with `joint_names`; `ee_targets` / `gripper` by (exposed) name; `base_velocity` / `base_target` / `latent` / `motion` to the one floating-base member (`SafetyError` otherwise). Every part is checked (`check_action`: fault, member gate, `direct_joints`, member `action_modes`) before any is sent; one refusal sends nothing. Members not mentioned get nothing and keep their last target; simulated ones advance one period (`idle_step`) |
| Capabilities | `action_modes`: modes every member takes. `direct_joints`, `cartesian_targets`: the members', prefixed. `motion_spaces`: the floating-base member's when there is exactly one |
| Errors | A member that fails to read or send, or reports a `fault`, makes the group hold every other member and enter its own fault state (`on_error: hold_all`). Then `send_action` raises `RobotFaultError` until `reset()` |
| Ownership | A member refuses `send_action` / `run_robot_skill` not coming through the group (`RuntimeError`). The mark is only on the object the group holds, so a whole-body-control layer's thread still commands its own inner robot |
| `member(name)` | Read-only view: `embodiment`, `simulated`, `fault`, `get_observation()`, `get_status()` |
| `get_status()` | `joint_temperature` concatenated; `joint_faults`, `board_temperature`, `extra` keys prefixed; `battery` top-level when one member reports one, else `extra["<member>/battery"]`; `fault` the group's or the first member's |
| Skills | Member skills listed as `<member>/<skill>`; `run_robot_skill("g1/wave")` runs it on that member |
| Lifecycle | `connect` connects members then scene cameras, undoing on failure. `reset`, `hold`, `damp`, `estop`, `disconnect`, `stop_robot_skill` reach every member even when one fails, then raise the first error. `reset` also clears the fault state |
| Gate | The group's own gate (`safety:` of the group config) checks that every number of a `members` action is finite; each member's gate checks its part |
| Collision | Optional `collision=CollisionCheck(...)` (config: `group.collision:`). Each modelled member is capsules, one per body-to-parent segment of its MJCF (`hirobot.kinematics`), all of `radius`; only gaps between different members count. Before sending, the group predicts each member's next pose (`POSITION` / `PD` joint targets as the next angles, otherwise the current ones; `CARTESIAN` target points as spheres against the other members) and refuses with `SafetyError` when the smallest gap would be under `min_distance`, unless it is not shrinking (members already too close may move apart). Reads each modelled member's observation once per send. A floating-base member needs a `base_pose`. About 1 ms for two G1s. Capsules along the chain are coarse: pick `radius` to cover the links; `skip:` leaves out bodies such as a mounting plate |
| Whole-body control member | A member wrapped in hiWBC's `wbc` layer (`layers: [{type: wbc, profile: ...}]` in its config) takes only `BASE_VELOCITY` / `MOTION_LATENT` / `MOTION_CLIP`, so the group's `action_modes` is usually empty: command it with `Action.members`, joint modes for the arms and a walk command for it. Its free joints (`direct_joints`, e.g. the upper body) may ride along by name in the same sub-action. When it is not named, `idle_step` runs one controller tick, so a simulated humanoid keeps standing. A controller error faults the member and holds the group. `tests/test_group_wbc.py` covers this with the `echo` profile (skipped without hiWBC) |
| Example | `groups/franka_ur5e_dual`: Franka FR3 (`left`) + UR5e (`right`) placed ±0.4 m apart, a scene camera exposed as `agentview`, end-effectors exposed as `left_arm` / `right_arm`. Both members use the toy `sim` driver unless overridden (`group.members.<name>.overrides`), so it runs without hardware |

Group config `collision:` section (paths relative to the group config file):

```yaml
group:
  collision:
    min_distance: 0.05        # m between capsule surfaces (default 0.05)
    radius: 0.05              # default capsule radius (m)
    backend: numpy            # or mujoco, for MJCFs the numpy parser rejects
    models:                   # members without a model are not checked
      left:  {mjcf: ~/models/fr3.xml}
      right: {mjcf: ~/models/ur5e.xml, radius: 0.06, skip: [base_mount]}
```


## Extension points: cameras, end-effectors and audio

Cameras, end-effectors (dexterous hands, grippers) and audio devices have their own registries, separate
from the body driver. Like body drivers, they import vendor SDKs lazily.

### Camera drivers (`hirobot.camera`, drivers in `hirobot.drivers.camera`)

| Name | Description |
|---|---|
| `register_camera(name)`, `get_camera(name)`, `available_cameras()` | Registry, same behavior as the driver registry |
| `CameraDriver(name)` | Abstract: `start()`, `latest() -> Optional[Frame]` (`(image, Stamp)`, `Stamp.t` on `time.monotonic()`, `seq` one more per new frame; must not block; `None` before the first frame), `stop()` (idempotent). RGB-D drivers also override `latest_rgbd() -> Optional[RgbdFrame]`; the default returns `latest()` with no depth |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | Runs a blocking `_grab() -> HxWx3 uint8 RGB` on a background thread and keeps only the newest frame. Subclasses implement `_grab` and optionally `_open` / `_close`. Exposes `wait_for_frame(timeout)`, `error`, `frames`, `errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`: the image and when it arrived (`Stamp(t, seq)`, `t` on `time.monotonic()`) |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`: colour image, depth registered to it (HxW float32, metres) or `None`, and their common `Stamp` |
| `agibot_cosine` → `AgibotCosineCamera(name, topic, depth_topic=None, depth_scale=0.001, max_pair_skew_s=0.005, pair_wait_s=0.02, queue=4, poll_hz=60.0)` | AgiBot G1 cameras on the robot's `cosine_bus` topics (`/camera/head_color` + `/camera/head_depth`, `/camera/hand_left_color`, ...). With `depth_topic` it is an RGB-D camera: each colour frame is paired with the depth frame of the same capture (`source_timestamp`), Z16 scaled to metres. Runs on the robot's computer with the vendor environment sourced (`LOCATOR_IP`, `AORTA_DISCOVERY_URI`) |
| `unitree_videohub` → `UnitreeVideohubCamera(name, network_interface=None, domain_id=0, timeout_s=1.0, poll_hz=30.0)` | G1 head camera |
| `unitree_image_server` → `UnitreeImageServerCamera(name, address, header=False, crop=None, timeout_s=1.0, poll_hz=60.0)` | H1 head camera: JPEG frames over ZeroMQ SUB from Unitree's teleoperation `image_server` (or `scripts/camera_image_server.py`). `header` strips the `Unit_Test` timestamp / frame id (kept in `remote_stamp` / `remote_frame_id`); `crop: [x, y, w, h]` takes one camera out of a side-by-side frame. Needs the `camera` extra |
| `video_file` → `VideoFileCamera(name, path, fps=5.0, loop=True)` | A video file played on its own timeline, sampled at `fps`. Without `loop` the last frame stays once the file ends (`ended` turns `True`). Needs the `video` extra (PyAV). Simulated |
| `array` → `ArrayCamera(name, frames=None, fps=10.0, advance_on_read=False)` | Cycles through frames in memory on the host clock; with no `frames`, one grey 64x64 frame. `advance_on_read=True` makes each `latest()` return the next frame (for tests). Simulated |
| `build_cameras(cameras, network_interface=None, sim_only=False) -> Dict[str, CameraDriver]` | One camera driver per camera with a `driver:`; `sim_only` builds only simulated ones. `make_robot` uses it for the embodiment's cameras |

A camera driver class sets `simulated = True` when it needs no hardware. Only
such drivers are built for a simulated body, and they do not get the body's
`network_interface`.

### End-effector drivers (`hirobot.end_effector`, drivers in `hirobot.drivers.end_effector`)

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

An end-effector driver is constructed as `cls(ee.name, joints, **spec)`, where
`joints` is empty for the `opening` interface and `spec` is the end-effector's
`driver:` dict without `type`. `make_robot` fills in the body's
`network_interface` when the spec sets none.

### Audio devices (`hirobot.audio`, drivers in `hirobot.drivers.audio`)

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
`joints`, `end_effectors`, `cameras`, `microphones` and `speakers` merge by
`name` (same name: fields merged; new name: appended); any other value is
replaced. A device's `driver:` merges key by key while its `type` stays the
same and is replaced whole when `type` changes. `hirobot info --config <name> --resolved` prints the result. Rules
and examples: `hirobot.config`.

```yaml
embodiment:                 # -> Embodiment (see above)
  name: unitree-g1
  kind: humanoid
  control_hz: 50
  floating_base: true
  joints:
    - name: left_hip_pitch_joint
    - {name: left_hand_thumb_0, position_limit: [-1.0472, 1.0472], torque_limit: 2.45}
  end_effectors:
    - name: left_hand
      joints: [left_hand_thumb_0, ...]
      driver: {type: unitree_handcmd, side: left}     # optional end-effector driver
    - name: gripper                                # a gripper with its own driver:
      has_gripper: true                            # no joints, so the 'opening' interface
      driver: {type: ...}
  cameras:
    - name: head
      width: 1920
      height: 1080
      driver: {type: unitree_videohub}             # optional camera driver
  microphones:                                     # optional audio devices
    - name: head
      driver: {type: unitree_voice_mic}
  speakers:
    - name: head
      driver: {type: unitree_voice_speaker, volume: 60}

body:
  driver: unitree_sdk2           # registered driver name; default: toy
  network_interface: eth0   # every other key -> driver constructor kwargs

safety:                     # optional; `safety: false` detaches the gate
  joint_limit: 3.0          # SafetyGate kwargs: joint_limit, clamp, gripper_range
  clamp: false

layers:                     # optional; registered layers, first innermost
  - type: my_wbc            # other keys -> the layer's constructor

sim:                        # optional MuJoCo twin for a real driver
  mode: mirror              # off (default) | mirror | shadow
  model: path/to/robot.xml  # required unless mode is off
  mirror_hz: 50
  viewer: true              # other keys -> MujocoRobot kwargs
```

Under `sim:`, `default_kp`, `default_kd`, `ramp_seconds` and `damping_kd` are
copied from `body:` when not set.

A robot group config has a `group:` section instead of `embodiment:` / `body:`
(`make_robot` refuses both in one file):

```yaml
group:
  name: dual_arm
  control_hz: 30              # optional; default: the slowest member's
  members:                    # name -> member; joint order follows this order
    left:  {config: franka_fr3, overrides: {ip: 172.16.0.2},   # overrides -> member's body:
            base_pose: [0, 0.4, 0, 0, 0, 0, 1]}      # optional; world [x, y, z, qx, qy, qz, qw]
    right: {config: ./my_ur5e.yaml, base_pose: [0, -0.4, 0, 0, 0, 0, 1]}  # relative: this file's dir
  cameras:                    # scene cameras, same format as embodiment.cameras
    - {name: scene, width: 640, height: 480, driver: {type: video_file, path: scene.mp4}}
  expose:                     # optional renames, e.g. to match training-data keys
    ee: {left/ee: left_arm, right/tcp: right_arm}
    cameras: {scene: agentview}
  on_error: hold_all          # the only value so far
safety: {}                    # the group's own gate; each member keeps its own
```

`make_robot(cfg, body_overrides={"right": {"ip": ...}})` merges each entry
into that member's overrides; `sim_overrides` goes to every member.

Policy settings (model server, chunk length) are not part of a robot config.

## CLI

Installed as `hirobot`.

| Command | Description |
|---|---|
| `hirobot --version` | Print the version |
| `hirobot info --config NAME_OR_PATH [--resolved]` | Print the embodiment: kind, control rate, floating base, joints, end-effectors, cameras. `--resolved` prints the whole config after `defaults:` are merged |
| `hirobot configs [GROUP]` | List the built-in configs (`robots`, `body`, `end_effector`, `camera`, `audio`, `sim`, `groups`). Names under `robots/` also work without the prefix (`unitree_g1`) |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | Run a simulated robot: connect, hold the start pose for N s (default 3), print the joint error left, the base height, the rendered images and `get_observation` time. Says when a free base fell. Exit 2 for a real body; `--viewer` needs a MuJoCo body |
