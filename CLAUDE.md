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
pip install -e '.[sim]'     # + mujoco  (mujoco driver, real-robot sim twin)
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
  `gripper_range` bounds are opt-in (robots differ, e.g. some grippers use `-1`). A
  refusal raises `SafetyError` from inside `send_action`; `clamp=False` by default, so
  out-of-range targets raise rather than being silently rewritten. `make_robot` attaches
  one from the embodiment + the config `safety:` section (`safety: false` detaches it).
- **`register_body` / `make_robot` (`registry.py`, `__init__.py`)** — name-based
  registries, plus external drivers from the `hirobot.drivers` entry-point group. Names
  are `<vendor>_<SDK or protocol>` and never change meaning once released.
- **Layers (`register_layer`, config `layers:`)** — a `Robot` wrapping another robot
  (hiWBC's whole-body controller). `make_robot` stacks them after the sim twin and
  before cameras/audio; with real end-effectors it first joins body + end-effectors so a layer sees
  every joint. A gate goes under the layers and another on top. hiRobot ships no layer.
  Capabilities callers read: `action_modes`, `motion_spaces`, `cartesian_targets`,
  `direct_joints` (joints a caller may name beside a non-joint mode); wrappers
  (`RobotAssembly`, `TwinRobot`) forward them explicitly, since base-class
  attributes never reach `__getattr__`.
- **Configs (`config.py`, `configs/`)** — built-in configs ship in the package, one file
  per part (`body/`, `end_effector/`, `camera/`, `audio/`) composed into `robots/*.yaml` with
  `defaults:`; named lists (`joints`, `end_effectors`, `cameras`, `microphones`,
  `speakers`) merge by `name`. `make_robot` / `load_embodiment` take a name, path or
  dict. `tests/data/expanded_configs.json` pins the expanded result.
- **`DofMap` (`dof.py`)** — reorder joint vectors by name, so one policy drives robots
  with different joint layouts.
- **`RobotGroup` (`group.py`, `collision.py`)** — several complete robots as one `Robot`
  (config `group:`). Checks every member's part before sending any; one failure holds
  all. Optional capsule distance check between members, built on `Kinematics`.
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

Setup guides live in `docs/setup/` (`<robot>.md`). `README.md`, `docs/api.md` and
each setup guide have a Simplified Chinese twin (`*.zh-CN.md`) with the same
headings, tables and code blocks; a change to one goes into both.

License (Apache-2.0), `src/hirobot` layout, and ruff line-length 100 mirror the sibling
hiVLA/hiAgent/hiWBC repos — match them. pytest is pinned to `tests/`. A `toy` driver ships
so the suite runs with no GPU or hardware. `tests/drivers/` mirrors `src/hirobot/drivers/`:
each SDK directory has a `fake_sdk.py` and a `test_contract.py` that runs the shared checks
in `tests/drivers/contract.py`; `tests/drivers/test_coverage.py` fails when a registered
driver has none. The rules for adding a model / SDK version / SDK are in the README's
"Add a new robot".
