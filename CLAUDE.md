# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

hiRobot is a **unified robot interface layer**: one `Robot` contract and one typed
I/O (`Observation` / `Action`) that control code is written against once and run
across many embodiments — humanoid or arm, simulation or real hardware — regardless
of the vendor SDK underneath. It standardizes the embodiment description (joints,
optional floating base, end-effectors, cameras, control rate) and the driver
contract. It does not run policies: the control loop and the VLA client live in
hiAgent (`hiagent.control`) and hiVLA (`hivla.client`).

Two sibling repos matter for context (all expected under `../`):

- **hiVLA** — the model serving layer (the motor cortex). Its client is
  `hivla.client`, used by hiAgent; hiRobot does not depend on hiVLA.
- **hiAgent** — the agentic operating layer that drives a hiRobot `Robot` in-process:
  the control loop (`run_policy`, `PolicyRunner`), `Policy` / `ChunkedPolicy`, the VLA
  adapter, and `hiagent eval`.

## Commands

```bash
pip install -e .            # core: numpy, pydantic, pyyaml
pip install -e '.[sim]'     # + mujoco  (mujoco driver)
pip install -e '.[dev]'     # + pytest, ruff

python -m pytest                              # full suite (no GPU/network needed)
python -m pytest tests/test_safety.py         # one file
python -m pytest tests/test_drivers.py::test_name   # one test
python -m pytest tests/drivers/body/unitree     # one vendor's drivers (fake SDKs)
ruff check .                                  # lint (line-length 100, target py39)

hirobot info --config unitree_g1    # show the embodiment a config describes
```

Run pytest **from the repo root**; `testpaths` pins it to `tests/`.

## Architecture

```
Robot driver (sim or real) ──▶ Observation ──▶ Policy ──▶ Action ──▶ Robot driver
                                                  │
                                                  └─ remote Policy → model server (hiVLA)
```

Core pieces:

- **`Embodiment` (`embodiment.py`)** — declarative robot I/O contract: joints by name,
  `floating_base`, end-effectors, cameras, `control_hz`. Drives DoF counts everywhere.
- **`Observation` / `Action` (`types.py`)** — typed I/O. Joint space (universal) +
  optional base (humanoids) + optional task space (arms) + perception (VLA).
  `ControlMode` selects joint vs Cartesian, or (for a robot that moves itself
  under a whole-body controller) `BASE_VELOCITY` / `BASE_POSE` / `MOTION_LATENT` /
  `MOTION_CLIP` (a `MotionClip`). Non-joint modes may carry targets for the joints
  the controller leaves free via `Action.joint_names`.
- **`Robot` (`robot.py`)** — driver contract: `connect` / `reset` / `get_observation` /
  `send_action` / `set_gains` / `disconnect`. The end-effector (hand / gripper),
  camera and audio contracts and registries are in `end_effector.py`, `camera.py`,
  `audio.py`. An end-effector driver has an interface: `joints` (lists joints, rides
  `joint_target`) or `opening` (no joints, `has_gripper: true`, rides
  `Action.gripper` / `Observation.gripper` in any mode). Concrete drivers live in
  `drivers/<part>/<SDK vendor>/<SDK or protocol>/` (`robots/unitree/sdk2/`,
  `end_effector/unitree/sdk2_handcmd/`, `cameras/unitree/sdk2_videohub/`, ...), each with a
  `driver.py` and a `manifest.yaml` that `connect()` checks via `require_sdk` (`sdk.py`);
  code shared inside one SDK is in `drivers/_sdk/`. Simulators are in `drivers/sim/` (sim
  in `toy.py`, mujoco; they set `simulated = True`); stand-in cameras
  and audio in `drivers/{camera,audio}/generic/`. Cameras with a `driver:`, end-effectors with a `driver:`, and
  microphones / speakers are attached by `make_robot` via `RobotAssembly` (on simulated
  drivers only the `simulated = True` ones); audio stays out of `Observation`, callers
  use `robot.microphones` / `robot.speakers`. Health
  signals (battery, motor temperatures / faults) also stay out of `Observation`: drivers
  override `get_status() -> RobotStatus` (default: empty), so they never ride to the
  model server. Robot skills (walk, gestures run by the robot's own controller)
  are the other way to command: `list_robot_skills` / `run_robot_skill` /
  `stop_robot_skill` (`robot_skills.py`), empty by default; `run_robot_skill` checks
  arguments against each skill's JSON Schema first. The G1's fixed list is in
  `drivers/body/unitree/sdk2/g1_skills.py` (`builtin_skills: g1`). `send_action` is a
  concrete template method that enforces the `SafetyGate` then calls the driver's
  abstract `_send_action` — **drivers override `_send_action`, never `send_action`**,
  so the gate is a choke point no caller can bypass.
- **`SafetyGate` / `SafetyError` (`safety.py`)** — the single enforced check before an
  action reaches hardware. Validates shape / DoF count / finiteness; `joint_limit` and
  `gripper_range` bounds are opt-in (robots differ, e.g. some grippers use `-1`). Stateful
  checks, all off by default: `max_joint_step` (from the last accepted target),
  `max_joint_error` (from the measured pose), `min_battery_soc` / `max_joint_temperature`
  (from `get_status`; refuse everything but `hold()`). Optional `sim_check` (`sim_check.py`,
  MuJoCo imported only then) rolls each command forward a few steps and refuses on
  collision / self-collision / torque; it only refuses, never clamps, and runs on the
  hardware gate only. A refusal raises `SafetyError` from inside `send_action`;
  `clamp=False` by default, so out-of-range targets raise rather than being silently
  rewritten. `make_robot` attaches one from the embodiment + the config `safety:` section
  (`safety: false` detaches it).
- **Command arbiter (`arbiter.py`)** — on the outermost robot of every `make_robot` stack.
  `robot.claim(name, priority, on_preempt=None, envelope=None)` returns a lease; while one
  is held, direct calls raise `RobotBusyError` (a `RuntimeError`). `TAKEOVER` >
  `LAYER_SKILL` > `CALLER`; a higher one preempts (one `hold()`, the old holder gets
  `PreemptedError`, no automatic return). `send_action` / `hold` / `reset` /
  `run_robot_skill` are checked; `estop` / `get_observation` / `get_status` / `now` are not.
  Nobody holding a lease means anyone may command, unless `arbiter: {require_claim: true}`.
  An `envelope` only tightens the outermost gate's limits. The audit keeps events
  (claim / release / preempted / refused / busy, `robot.command_events()`) and per-lease
  counts (`get_status().safety["audit"]`), not every command. A part of an assembly is a
  `Robot` with its own lease, claimed by path (`robot.claim(name, parts=["left"])`);
  overlapping paths conflict, disjoint ones don't.
- **Command watchdog (`watchdog.py`)** — `Robot.command_timeout_s` (0.2 s default, `None`
  off): a `VELOCITY` / `BASE_VELOCITY` command not refreshed in time is stopped with
  `hold()`. Position targets are not watched.
- **`register_body` / `make_robot` (`registry.py`, `__init__.py`)** — name-based
  registries, plus external drivers from the `hirobot.drivers` entry-point group. Names
  are `<vendor>_<SDK or protocol>` and never change meaning once released.
- **Layers (`register_layer`, config `layers:`)** — a `Robot` wrapping another robot
  (hiWBC's whole-body controller); subclass `Layer` (`layer.py`), which forwards
  everything to `inner`. `make_robot` builds one `RobotAssembly` (body, end-effectors,
  cameras, audio), attaches the gate to it, then stacks the layers on top, so a layer
  sees every joint and device; with layers a second gate goes on top. A layer with
  `part: <path>` wraps only that part of an assembly (`part_layers.py`), sitting under the
  whole-robot layers. hiRobot ships no layer.
  Layers are trusted code: `Layer.inner` is a proxy with the public API only (no `_` names,
  no `body` / `end_effectors`); `Layer.estop()` e-stops the bottom of the stack first, then
  each layer's `_estop()` (0.2 s each). `hirobot.testing.assert_layer_uses_public_api` checks
  a layer. SPI methods are `_`-prefixed: `_set_safety`, `_set_fault`, `_clear_fault`,
  `_idle_step`; applications never call them (`docs/api.md` keeps them in "Writing a
  driver or a layer (SPI)"). Layer skills: `list_layer_skills` / `run_layer_skill` /
  `stop_layer_skill`; while one runs the layer holds the lease with `LAYER_SKILL`. A layer
  implements `_layer_skills` / `_start_layer_skill` / `_stop_layer_skill`.
  Capabilities callers read: `action_modes`, `motion_spaces`, `cartesian_targets`,
  `direct_joints` (joints a caller may name beside a non-joint mode); wrappers
  (`RobotAssembly`, `Layer`) forward them explicitly, since base-class
  attributes never reach `__getattr__`.
- **Configs (`config.py`, `tree.py`, `configs/`)** — a config is a **device file**
  (top-level `device:` — `body`, `end_effector`, `camera`, `microphone`, `speaker`) or a
  **robot config** (`assembly:` listing `parts:`). A part is a device or another node,
  referenced with `from:` (built-in name or path) or written inline; `placement:` says
  where each sits. Built-ins ship one file per device under `body/` etc., composed into
  `robots/*.yaml`. `tree.py` loads the tree and merges `overrides` level by level;
  `parts.py` works out each part's frame and the merged `Embodiment` (names under the
  `<part>/` prefix). `defaults:` still merges whole configs; named lists (`joints`,
  `frames`, `mounts`, device lists) merge by `name`. `make_robot` / `load_embodiment`
  take a name, path or dict. The old `embodiment:` + `body:` single-robot format still
  loads through `legacy.py` with a `DeprecationWarning`; `group:` is rejected.
  `tests/data/expanded_configs.json` pins the expanded result.
- **`DofMap` (`dof.py`)** — reorder joint vectors by name, so one policy drives robots
  with different joint layouts.
- **Assembly with parts (`assembly.py`, `node.py`, `collision.py`)** — a `RobotAssembly`
  with child parts (`parts=`, `body=None`) drives several complete robots as one `Robot`
  (config `assembly:` with a `parts:` tree); parts nest. `node.py` concatenates joints,
  prefixes keyed fields, converts `ee_poses` into the node's frame by each part's
  `placement`, and splits an `Action` (flat, or per part via `Action.parts`). Checks every
  part's piece before sending any; one failure holds all. Optional capsule distance check
  between parts, built on `Kinematics`.
- **`hirobot view` (`view.py`)** — outside the stack: opens a real config with
  `read_only: true` (refuses a driver without that option) and copies its measured pose
  into the `<config>_mujoco` model each frame with `set_state`. Never commands. For
  rehearsal in sim either build two robots by hand (`examples/rehearse_in_sim.py`) or set
  `sim:` / pass `sim=True`, which builds the whole config in one shared MuJoCo world
  (`sim_world.py`).
- **`Kinematics` (`kinematics.py`)** — forward kinematics of an MJCF: numpy by default,
  `backend="mujoco"` (`mj_kinematics`) only when asked; the tests check the two agree.
- **What a control loop needs from a driver** (the loop itself is in hiAgent):
  `reset(seed=None)` (ramps gains / homes / reseeds the scene), `get_observation`,
  `send_action`, `hold()` (keep still after a stop; a driver with its own balance loop
  must override it to command standing still, not the current `joint_pos`), and on
  simulators `success` / `done` / `reward`.

**Invariants to preserve when changing any of this:**

- **Drivers are interchangeable behind the `Robot` contract.** Sim and real differ only
  by which driver a config selects; don't leak driver-specific assumptions up into the
  types or into hiAgent's control loop.
- **Drivers fill every observation field they have** — including base signals and
  `extra` (where a driver parks sensors beyond the core); hiAgent's VLA client forwards
  them all, so the model server sees exactly what the robot provides. Keep health
  signals in `get_status`, never in `Observation`.
- **Safety stays below the layers.** The hardware gate sits on the assembly under every
  layer, `estop()` reaches the bottom of the stack before any layer's `_estop()`, and a
  layer's `inner` exposes the public API only. Don't add a path from a layer, the arbiter
  or a caller to a driver that skips them.
- **DoF order is resolved by name, not position.** When joint layouts differ, route
  through `DofMap`; never assume a fixed index order across embodiments.
- **`Observation.base_orientation` is `[x, y, z, w]`.** Drivers whose SDK reports
  `[w, x, y, z]` (Unitree, MuJoCo) reorder it.
- **Heavy / vendor deps are optional and lazy.** The core is numpy + pydantic + pyyaml;
  `mujoco` (sim), `opencv-python` (camera), `av` (video) and `sounddevice` (audio)
  are extras, imported where used. Keep new vendor-SDK driver deps lazy so the package imports without them.

## Conventions

See `CONTRIBUTING.md` for the contribution workflow. Never commit machine-specific
details (local paths, NIC names, environment names, lab IPs) to shared files; local
configs inherit a built-in one with `defaults:`.

Setup guides live in `docs/setup/` (`<robot>.md`). `README.md`, `CHANGELOG.md`, `docs/api.md`
and each setup guide have a Simplified Chinese twin (`*.zh-CN.md`) with the same
headings, tables and code blocks; a change to one goes into both.

License (Apache-2.0), `src/hirobot` layout, and ruff line-length 100 mirror the sibling
hiVLA/hiAgent/hiWBC repos — match them. pytest is pinned to `tests/`. A `toy` driver ships
so the suite runs with no GPU or hardware. `tests/drivers/` mirrors `src/hirobot/drivers/`:
each SDK directory has a `fake_sdk.py` and a `test_contract.py` that runs the shared checks
in `tests/drivers/contract.py`; `tests/drivers/test_coverage.py` fails when a registered
driver has none. The rules for adding a model / SDK version / SDK are in the README's
"Add a new robot".
