# Changelog

User-facing changes in each hiRobot release, newest first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Before 0.1.0, any
release may change the API incompatibly; such changes are listed under
"Removed" or "Changed" and marked **breaking**.

## [0.0.8] - 2026-11-27

Simulation for more robots: the Unitree H1 with its Inspire hands, the Fourier
GR2 and the Trossen ALOHA run in MuJoCo, and cameras on arm, H1 / H2 and URDF
models no longer render black. The README introduces hiApp, the web interface.

### Added

- **`joint_map`** on an end-effector (simulation only): each hand joint drives a list of
  model joints, each to `offset + scale * value`, and reads back from the first. The
  Inspire RH56DFX hands use it, so `unitree_h1_inspire` runs in MuJoCo with
  `h1_with_hand.xml` (12 finger joints per hand, normalised 0 = closed, 1 = open).
  `safety.sim_check` refuses a robot with a `joint_map` for now.
- **`model: {joint_names: {old: new}}`** renames MJCF joints (and the actuators naming
  them) before the model compiles.
- **`hirobot.sim.world_spec.world_spec(config)`**: the composed, uncompiled MjSpec of every
  body of a config (prefixes, placement poses, `fixed_base`), without the default stage,
  plus its joint names. For viewers of multi-body configs.
- New configs `fourier_gr2` and `trossen_aloha`. Default camera placements for
  `agilex_piper`, `franka_fr3_droid`, `franka_ur5e_dual` and `trossen_wx250s` (wrist camera).
- README: a Web interface section on
  [hiApp](https://github.com/humanoidintelligence/hiApp), which drives any hiRobot
  config from a browser.

### Changed

- A simulation world whose models bring no floor, light or sky gets a default stage
  (`hirobot.sim.stage`). Cameras on Menagerie arms, H1 / H2 and URDF models rendered
  black before.
- The shared simulation world compiles a URDF-only body too (adding position motors);
  `safety.sim_check` still needs an MJCF.
- Actuators no view drives (H1 wrist joints, Menagerie grippers) hold their start pose with
  a fixed PD instead of getting zero torque.

## [0.0.7] - 2026-11-26

Episode recording as LeRobot datasets, an IK layer for end-effector targets, an
MCP server, a generic ROS 2 body driver, camera intrinsics, and simulation
models for every built-in body except AgiBot, downloaded with
`hirobot assets fetch`. Arms (UR5e, PiPER, WidowX 250S) can now run in MuJoCo.

### Added

- **Recording** (`hirobot.record`, `pip install 'hirobot[record]'`):
  `Recorder(robot, root)` records episodes (`start(task)` / `stop()` /
  `cancel()`) and writes each one as a LeRobot v3.0 dataset (parquet + H.264
  video) without importing `lerobot`. State, commands and images are aligned to
  a frame grid; save-time quality flags go to `meta/qc.jsonl`, and
  `append_verdict()` adds a human verdict. Episodes are saved in a child
  process, so the control loop is barely slowed while one saves. Example:
  `examples/record_g1.py` on the MuJoCo G1.
- **Listeners**: `robot.add_listener(on_observation=..., on_command=...)`
  reports every top-level `get_observation()` and every top-level
  `send_action` / `hold` as a `CommandRecord`, refusals included, without
  sitting in the command path. `hirobot.core.features` (`observation_features`,
  `flatten_observation`, `action_features`, `flatten_action`,
  `image_features`) flattens `Observation` / `Action` into named vectors in the
  LeRobot layout.
- **`mink_ik` layer** (`pip install 'hirobot[ik]'`, Python 3.11+): turns
  `CARTESIAN` / `CARTESIAN_DELTA` end-effector targets in the base frame into
  one `POSITION` action, solved by mink IK on a MuJoCo model (the
  simulation's own, an MJCF, or a config name). The floating base stays
  frozen; a solver failure resends the previous target. Example:
  `examples/mink_ik_g1.py`.
- **MCP server** (`pip install 'hirobot[mcp]'`, the server needs Python 3.10+):
  `hirobot mcp <config>` or `serve_mcp(robot)` serves a robot's tools
  (describe, observe, status, act, hold, stop, skills, reset) over stdio or streamable HTTP, set up
  by the config's `mcp:` section. HTTP listens on 127.0.0.1 by default.
  Example: `examples/mcp/`; guide: `docs/mcp.md`.
- **Generic ROS 2 body driver** (`ros2`): a ROS 2 robot is driven from its
  config alone. State components (`joint_state`, `imu`, `odometry`) and command
  components (`joint_trajectory`, `float_array`, or a custom message filled
  from a field map) list the topics; joints are matched by name. Guide:
  `docs/setup/ros2.md`.
- **Camera intrinsics and extrinsics**: `Robot.camera_info(name)` returns a
  `CameraInfo` (`CameraIntrinsics`, where they came from, the mount pose,
  depth). Intrinsics come from the camera file's `intrinsics:`, else the
  driver (RealSense, Orbbec and ZED read them from the device), else an ideal
  pinhole from the mount's `fovy`, rescaled to the declared image size.
- **Speaker volume**: speakers may support `volume` / `set_volume(level)`
  (0-100), shown by `supports_volume`. The Unitree G1 speaker sets the
  hardware volume and restores it on close; the WAV, null and sounddevice
  speakers apply a software gain.
- **Simulation models** (`pip install 'hirobot[assets]'`):
  `hirobot assets list` and `hirobot assets fetch NAME | --all` download the
  MJCF / URDF models from the `humanoidintelligence/hirobot-assets` dataset on
  Hugging Face into `~/.cache/hirobot/assets`. `HIROBOT_ASSETS_REPO` picks
  another dataset with the same layout.
- Every built-in body except the AgiBot G1 and X1 now has a `model:`: UR5e,
  PiPER and WidowX 250S (MuJoCo Menagerie), Unitree H1, H1-2 and H2, Franka
  FR3, and Fourier GR1T1 / GR1T2 (URDF only). New configs `ur_ur5e_mujoco`,
  `agilex_piper_mujoco` and `trossen_wx250s_mujoco` run those arms in MuJoCo
  (`hirobot sim <config>`); the gripper does not move in simulation.
  `docs/supported_robots.md` lists where each model comes from.
- An `all` extra: `pip install 'hirobot[all]'` installs every optional feature
  (about 1 GB). The README's install line uses it; the core install is
  unchanged.
- **Model paths**: `Embodiment.model` (a `ModelRef`: `kind` mjcf / urdf,
  `path`, `package_roots`) and `hirobot.model_path(config)` give a body's model
  file without building the robot; `hirobot.available_configs(group)` lists the
  built-in config names. A device file's `model:` may give a `urdf:` (with
  `package_roots`) instead of an `mjcf:`. `hirobot info` prints the model path.
- **`hirobot scan` and `hirobot init`**: `scan` looks at this computer and the
  robot network read-only (network cards, the Unitree subnet, installed driver
  SDKs, ROS 2, cameras, audio) and prints a report with suggestions. `init`
  scans, writes a local config for a Unitree robot and checks it read-only.
- **Per-model health thresholds**: `Joint.temperature_limit` and
  `Embodiment.battery_soc_limit`, each an optional `(warn, shutdown)` pair. When
  `safety:` sets no `max_joint_temperature` / `min_battery_soc`, the
  `SafetyGate` takes the shutdown value from the embodiment.
- **Unitree G1 `turn` skill**: turns on the spot by an angle, closed-loop on
  the IMU yaw. `walk` and `turn` now report the expected duration of each run
  in `duration_s`. Not yet tried on a real G1.

### Changed

- **Breaking**: the AgiBot X1 now uses the `ros2` driver; `agibot_aimrt` and
  `AgibotX1Robot` are removed. `/joint_states` is matched by joint name
  instead of message order, and `spin_timeout` is now `connect_timeout_s`.
- **Breaking**: the Unitree G1 model moved from
  `~/.cache/hirobot/unitree_ros/...` to `~/.cache/hirobot/assets/unitree_ros/...`
  (`hirobot assets fetch unitree_g1`). Local configs that point at the old
  path need updating.
- Simulated cameras render at their own frame rate (`Camera.fps`, else
  `sim.camera_fps`, default 30) and repeat the last image in between, as a real
  camera does. A camera's `stamps[...].seq` counts its own frames.
  `camera_fps: null` renders on every observation, as before.
- `sim_check` refuses a body whose `model:` has only a `urdf`, when the robot
  is built.
- The `mink_ik` layer's `model:` may name a config (`.yaml`) as well as an MJCF.
- The H1 setup guide no longer asks you to edit the MJCF by hand: the dataset
  ships `h1_hirobot.xml` with the waist joint renamed.

### Fixed

- The MuJoCo and toy simulators fill `base_linear_acceleration` as an IMU
  reads it (specific force in the base frame, gravity included); it was
  missing.
- Unitree G1 gestures longer than 3 s (such as `high_wave`) no longer report
  "refused (RPC code 3104)" while the robot performs them.
- `unitree_g1_dex3` now carries the G1 model with both Dex3-1 hands. It had
  the bare 29-DoF model, so the 3D view drew no hands and
  `make_robot(..., sim=True)` failed on the 43 joints.

## [0.0.6] - 2026-11-15

Trossen WidowX 250S support, with its gripper and RealSense D435i cameras. The
project subtitle is shorter.

### Added

- **Trossen WidowX 250S arm** (new driver `trossen_dynamixel`): talks to the
  arm's Dynamixel servos directly with Robotis' `dynamixel_sdk` over a U2D2
  (Protocol 2.0, 1 Mbit/s), with no ROS and no Interbotix stack. It reads
  joint positions and velocities, the gripper opening, motor temperatures, bus
  voltage and per-servo torque and error state. It commands the six joints in
  `ControlMode.POSITION` and the gripper through `Action.gripper["ee"]` (0
  closed, 1 open). `connect()` and reading enable no torque and send no goal,
  so the arm stays limp until the first `send_action`. That call enables torque
  at the current position, so the arm does not jump. Configs
  `body/trossen/wx250s` and `robots/trossen_wx250s` (the arm, its gripper and
  two RealSense D435i cameras, wrist and scene). Verified on a real WidowX
  250S: readout, gripper calibration and commands, POSITION moves of single
  joints, and one D435i streaming colour and depth. `estop()` holding a moving
  arm has not been tried on hardware.
- `docs/setup/trossen_wx250s.md`: a setup guide for the WidowX 250S
  (prerequisites, power-on and safety, configuration and gripper calibration,
  observation and action, a minimal loop, cameras, troubleshooting).

### Changed

- The project subtitle is now "A Unified Interface Layer for Controlling
  Diverse Robots" (it was "... from VLA Models and Agents"), in the README
  title, the citation, the package description and `hirobot --help`. Update
  the title if you cite hiRobot.
- The WidowX 250S is marked hardware-verified (✅) in the README and
  `docs/supported_robots.md`, and links to its setup guide. The photo wall gets
  the WidowX 250S gripper. The end-effector table there gets a WidowX 250S
  gripper row, and the RealSense D435i camera row lists `trossen_wx250s`.

## [0.0.5] - 2026-11-14

The skill methods are renamed. Joints can describe their rotation axis and
parent link, and the Unitree G1's joints now carry limits, torque limits and
axes.

### Added

- `Joint.axis` and `Joint.parent_link`, both optional and allowed in body
  configs. `axis` is the unit vector the joint turns about, in the frame of the
  link the joint moves (as URDF `<axis>` and MJCF `axis`), not the world frame.
  `parent_link` is the link the joint hangs from (URDF `<parent>`).
- The Unitree G1 body (`body/unitree/g1`) gives every one of its 29 joints a
  `position_limit`, `torque_limit`, `axis` and `parent_link`, taken from
  `g1_29dof_rev_1_0.urdf` in Unitree's `unitree_ros`. Nothing in hiRobot clips
  commands to these per-joint limits; they describe the robot.

### Changed

- **breaking**: the skill methods are renamed. The one list of every skill
  (layer skills and the robot controller's own) is now "robot skills"; the
  skills the robot's own controller runs are now "onboard skills". Layer
  skills and the `RobotSkillSpec` / `RobotSkillResult` / `RobotSkillError`
  types keep their names.

  | 0.0.4 | 0.0.5 |
  |---|---|
  | `list_skills()` | `list_robot_skills()` |
  | `run_skill(name, args)` | `run_robot_skill(name, args)` |
  | `stop_skill()` | `stop_robot_skill()` |
  | `skill_running` | `robot_skill_running` |
  | `has_robot_skills` | `has_onboard_skills` |
  | `list_robot_skills()` | `list_onboard_skills()` |
  | `run_robot_skill(name, args)` | `run_onboard_skill(name, args)` |
  | `stop_robot_skill()` | `stop_onboard_skill()` |
  | `_run_robot_skill` (driver SPI) | `_run_onboard_skill` |

  `CommandLease.run_skill` is now `run_robot_skill`, and
  `CommandLease.run_robot_skill` is now `run_onboard_skill`.

  **Watch out:** `list_robot_skills`, `run_robot_skill` and `stop_robot_skill`
  still exist but now mean the whole list. Code written for 0.0.4 that calls
  them gets layer skills too, with no error. A driver that overrides
  `list_robot_skills` or `_run_robot_skill` must rename them to
  `list_onboard_skills` / `_run_onboard_skill`; otherwise its skills no longer
  show up.
- `Joint` now refuses a `position_limit` whose lower bound is above the upper,
  a `torque_limit` that is not above 0, and an `axis` that is not a unit
  vector, so a body config with such values fails to load. Every built-in
  config passes.
- `docs/setup/unitree.md`: which FSM state the G1 onboard skills need (gestures
  only in 500, 501 or 801), a stand-up sequence verified on one G1, that
  `SetFsmId` returns 0 even when ignored, the arm error codes 7400-7404, and
  why the head camera stays down after `videohub_pc4` fails to start 30 times.

## [0.0.4] - 2026-11-13

Franka FR3 and AgileX PiPER support, with ZED and Orbbec cameras. The Franka
FR3 now runs through a DROID robot server; the libfranka driver is removed.

### Added

- **AgileX PiPER arm** (new driver `agilex_piper_sdk`): AgileX's `piper_sdk`
  over SocketCAN. Joints, the flange pose and the gripper; POSITION joint moves
  planned by the arm's controller at a set speed, `Action.gripper` for the
  gripper. `connect()` and reading send nothing on the bus. Configs
  `body/agilex/piper` and `robots/agilex_piper` (arm, gripper and an Orbbec
  scene camera). Verified on a real PiPER: readout, a 10 cm lift and back, and
  the gripper. `estop()` is not yet checked on hardware.
- **Orbbec cameras** (new driver `orbbec_pyorbbecsdk`): the 1.x `pyorbbecsdk`
  bindings over local USB, picked by USB port or serial; colour, with optional
  depth in metres aligned to colour. Config `camera/orbbec/dabai_dc1`. Verified
  on a Dabai DC1 at 640x480 and 1280x720, 30 fps with depth.
- **Franka arms on a DROID station** (new driver `franka_droid`): a client of
  [DROID](https://github.com/droid-dataset/droid)'s robot server (zerorpc,
  port 4242), using only the server methods upstream DROID has, so it needs no
  RT kernel or libfranka on the hiRobot side, only `pip install zerorpc`.
  Joints, end-effector pose and gripper readout; POSITION (`update_joints`) and
  CARTESIAN (`update_pose`) control, `Action.gripper` for the gripper. Configs
  `body/franka/fr3_droid` and `robots/franka_fr3_droid` (arm plus three ZED
  cameras). Verified on a real FR3 with a Robotiq 2F-85: readout, a CARTESIAN
  move 10 cm up and back, and the gripper. POSITION, `reset(home_q)` and
  `estop` are not yet checked on hardware.
- **Stereolabs ZED cameras** (new driver `stereolabs_zed`): pyzed over local
  USB, picked by serial number; the left camera's colour, with optional depth
  in metres registered to it. Configs `camera/stereolabs/zed2i` and
  `camera/stereolabs/zed_mini`. Verified on two ZED 2i and a ZED Mini running
  at once (HD720 at 30 fps with depth).
- **Skills in one list**: `Robot.list_skills()`, `run_skill(name, args)`,
  `stop_skill()` and `skill_running`, on every robot, list and run robot skills
  and layer skills together, so a caller need not tell them apart. A layer
  skill hides a robot skill of the same name; in an assembly names carry the
  part path. `CommandLease.run_skill` too. The eight per-kind methods stay,
  for drivers and layers. See [Skills](docs/api.md#skills).
- `docs/setup/agilex_piper.md`: a setup guide for the PiPER and its Orbbec
  camera. `docs/setup/franka.md` now covers only the DROID robot server.
- `hirobot.geometry.transforms`: `euler_xyz_to_quat` and `quat_to_euler_xyz`
  (extrinsic `xyz` Euler angles, scipy's `"xyz"`).

### Changed

- Franka FR3, AgileX PiPER and its gripper, the Orbbec Dabai DC1 and the ZED
  2i / ZED Mini are marked verified (✅) in the README and
  `docs/supported_robots.md`, with photos on the hardware wall.
- `docs/api.md` is reordered: a new first section, "Robot at a glance", groups
  every `Robot` member by use; then the quick example, `make_robot`, `Robot`
  and the data types, the developer sections (driver SPI, extension points)
  last. The `Robot` table also lists `cameras` / `microphones` / `speakers`,
  `accepts_parts`, `claim` and `command_events`.
- The README has a doc index at the top, next to the language switch.

### Removed

- **breaking**: the `franka_libfranka` driver (`FrankaRobot`, over
  `pylibfranka`) and the `franka_fr3` / `body/franka/fr3` configs. It never ran
  on a real arm. A Franka FR3 is driven through a DROID robot server
  (`franka_droid`, `franka_fr3_droid`). The left arm of `franka_ur5e_dual` is now
  `body/franka/fr3_droid`; its names (`left/joint1`, `left/ee`) are unchanged.

## [0.0.3] - 2026-11-12

UR arm support, with two new drivers: Robotiq grippers and Intel RealSense
cameras. No breaking changes.

### Added

- **UR arms on hardware.** The `ur_rtde` driver is verified on real UR5 (CB3)
  arms, both a single arm and the two-arm `ur5e_dual` assembly: joint and TCP
  pose readout, and CARTESIAN control (`servoL`) at `control_hz: 125`.
  `reset` / `estop` and the POSITION / PD / VELOCITY modes are not yet checked
  on hardware.
- `body/ur/ur5e` has a `flange` mount (`wrist_3_link`) for end-effectors.
- **Robotiq 2F grippers** (new driver `robotiq_urcap`): talks to the Robotiq
  URCap on the UR control box over its socket (port 63352), so it needs no
  extra wiring or Python package. `opening` interface (0 closed, 1 open).
  `activate` is off by default, since activation runs the gripper through a
  full open-close cycle. Configs `end_effector/robotiq/2f85_urcap` and
  `robots/ur_ur5e_robotiq`. Verified on a real 2F-85.
- **Intel RealSense cameras** (new driver `intel_realsense`): pyrealsense2 over
  local USB, color plus depth aligned to color. `depth_width` /
  `depth_height` can differ from the color resolution (an L515 on USB 2 gives
  only 320×240 depth). Configs `camera/intel/d435i_realsense` and
  `camera/intel/l515_realsense`. Verified on a D435i (pyrealsense2 2.59.1)
  and an L515 (2.54.2; newer SDKs do not list the L515), both at 30 fps at
  the same time.
- `RobotStatus.command` (`CommandProgress`: `mode`, `done`, `goal`,
  `clip_time`, `clip_duration`): progress of a high-level command
  (BASE_POSE, MOTION_CLIP), reported up by a layer. In an assembly it sits at
  the top level when only one part reports it, otherwise in
  `extra["<part>/command"]`. hiRobot's own drivers leave it `None`.
- `docs/setup/ur.md`: hardware status, the two-arm config, RealSense cameras,
  the Robotiq gripper and the L515.
- `docs/api.md`: a `RobotStatus` section, `get_status` in the `Robot` table,
  RealSense and Robotiq in the driver tables.

### Changed

- The four layer-skill methods (`list_layer_skills`, `run_layer_skill`,
  `stop_layer_skill`, `layer_skill_running`) are on the `Robot` base class, so
  a robot without layers answers them with defaults.
- UR5e, Robotiq 2F-85 and RealSense L515 are marked verified (✅) in the
  README and `docs/supported_robots.md`; the camera table there lists
  "D435i / D405" and "L515" as separate rows.
- The UR5e MuJoCo tests work with both the old and the current MuJoCo
  Menagerie model (joint names and base orientation differ).

### Deprecated

- The old single-robot config format (`embodiment:` + `body:`) still loads
  with a `DeprecationWarning`; it is planned to go in the next release.

## [0.0.2] - 2026-11-11

Nested assembly trees and a reworked layer stack.

### Added

- **Assembly trees.** A robot config is an `assembly:` tree: each node is a
  device file or a complete single-robot config, and `make_robot` builds the
  whole tree as one `RobotAssembly`. All built-in configs use it; new configs
  `franka_ur5e_dual` and `ur5e_dual`.
- A `RobotAssembly` can have child parts: several complete robots driven as
  one `Robot` in a shared frame. `robot.part("left")` returns one part;
  `placement:` is resolved per part; `overrides={"left/body": ...}` overrides
  one part's config.
- Leases on parts (`claim(..., parts=[...])`) and layers on one part
  (`layers: [{part: left, ...}]`).
- The whole tree in one shared MuJoCo world (`sim:`); the safety gate's
  simulation check is built from the tree (`safety.sim_check`);
  `hirobot check --sim [--against-real]`.
- The `toy` body can act as a mobile base (`mobile_base: true`).
- **Command arbiter** on the outermost robot: `robot.claim(name, priority)`
  returns a lease; `TAKEOVER` > `LAYER_SKILL` > `CALLER`, a higher one
  preempts; a lease may carry a tighter `envelope`; claims, preemptions and
  refusals are audited (`robot.command_events()`).
- **Command watchdog**: a velocity command not refreshed within
  `command_timeout_s` is stopped.
- `Layer` base class and layer skills (`run_layer_skill`, which holds the
  lease at `LAYER_SKILL` while it runs); "Writing a layer" in the docs.
- Safety gate: step and tracking-error limits, battery and temperature
  checks, per-joint limits, per-gripper ranges, optional MuJoCo look-ahead
  check.
- `hirobot view`: shows a real robot's measured pose in MuJoCo, read-only.

### Changed

- The stack, bottom up: drivers → one `RobotAssembly` → hardware
  `SafetyGate` → layers → command arbiter.
- `estop()` reaches the bottom driver first, then each layer cleans up; a
  layer sees only the public API of its `inner`.
- **Breaking:** driver and layer hooks (SPI) are `_`-prefixed.
- **Breaking:** the modules under `src/hirobot` are grouped into `core/`,
  `devices/`, `geometry/`, `loading/`, `assembly/`, `control/`, `sim/`.
  `from hirobot import ...` is unchanged; deep imports moved (for example
  `hirobot.kinematics` → `hirobot.geometry.kinematics`).
- **Breaking:** `Action.members` is renamed `Action.parts`.
- README: a hardware photo wall; the supported-robot list moved to
  `docs/supported_robots.md`; quick start before the architecture; a new
  architecture diagram. `docs/api.md` is split into the application API and
  the SPI.

### Deprecated

- The old single-robot config format (`embodiment:` + `body:`) still loads
  through `hirobot.loading.legacy` with a `DeprecationWarning`.

### Removed

- **Breaking:** `RobotGroup` and `group:` configs; use a `RobotAssembly` with
  child parts.
- **Breaking:** the sim twin (`TwinRobot`, mirror / shadow, `sim_overrides`);
  use `hirobot view` to watch a real robot.

## [0.0.1] - 2026-11-09

First public release: one `Robot` interface for humanoids and arms, in
simulation and on real hardware. Control code, VLAs and agents are written
once against `Observation` / `Action` and run on any robot with a driver.

### Added

- **Robot contract**: `connect` / `reset` / `get_observation` /
  `send_action` / `set_gains` / `disconnect`, plus `get_status` (battery,
  temperatures, joint faults), `estop()` that interrupts a running `reset` or
  `connect` from any thread, and `action_modes` / `motion_spaces` listing
  what a driver accepts. `Observation.stamps` gives each source (body, hands,
  cameras) its own timestamp and sequence number.
- **Safety**: every action passes a `SafetyGate` (shapes, finite values,
  joint and gripper ranges) before it reaches a driver; a refused action
  raises `SafetyError`.
- **Configs**: robots described in YAML (joints by name, floating base,
  end-effectors, cameras, control rate), composed from one file per part
  with `defaults:`. `make_robot("unitree_g1")` builds the full stack.
  `hirobot info`, `hirobot configs` and `hirobot sim` on the command line.
- **End-effectors**: dexterous hands and grippers share one driver contract.
  Hands are joints in `joint_target`; grippers are an opening in [0, 1]
  (`Observation.gripper` / `Action.gripper`, 0 closed, 1 open) in any
  control mode.
- **Simulation**: a MuJoCo robot with the real robot's names, joint order and
  cameras, on its own, as a `mirror` twin of a real robot, or as a `shadow`
  that rehearses a motion from the real robot's current pose. A
  dependency-free `toy` driver for tests.
- **Several robots as one**: `RobotGroup` combines complete robots (different
  vendors) and scene cameras into one `Robot` in a shared world frame, with a
  distance check between members.
- **Extension points**: register body drivers, end-effectors, cameras, audio
  devices and `layers:` (such as a whole-body controller) from your own
  package through the `hirobot.drivers` entry point.
- **Robot skills**: built-in vendor motions (G1 walking, arm gestures)
  started by name, each with a description and a JSON Schema for its
  arguments.
- Verified on real robots: Unitree G1 and H1 bodies; Unitree Dex3-1 hands
  (G1); Inspire RH56DFX hands (H1); AgiBot G1 body, both Omnipicker grippers
  and chassis, its head (RealSense D457) and wrist (RealSense D405) cameras;
  RealSense D435i head cameras on the Unitree G1 and H1. Drivers for Fourier
  GRx, AgiBot X1, Franka FR3 and UR5e are included but not yet checked on
  hardware.

[0.0.8]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.7...v0.0.8
[0.0.7]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.6...v0.0.7
[0.0.6]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.5...v0.0.6
[0.0.5]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.4...v0.0.5
[0.0.4]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.3...v0.0.4
[0.0.3]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.2...v0.0.3
[0.0.2]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.1...v0.0.2
[0.0.1]: https://github.com/humanoidintelligence/hiRobot/releases/tag/v0.0.1
