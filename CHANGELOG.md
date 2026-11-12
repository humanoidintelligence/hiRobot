# Changelog

User-facing changes in each hiRobot release, newest first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Before 0.1.0, any
release may change the API incompatibly; such changes are listed under
"Removed" or "Changed" and marked **breaking**.

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

[0.0.4]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.3...v0.0.4
[0.0.3]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.2...v0.0.3
[0.0.2]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.1...v0.0.2
[0.0.1]: https://github.com/humanoidintelligence/hiRobot/releases/tag/v0.0.1
