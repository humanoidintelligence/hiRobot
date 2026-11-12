"""hiRobot — a unified interface layer for controlling diverse robots from VLA models and agents.

hiRobot standardizes how a model or agent observes and commands a robot —
humanoid or arm, in simulation or on real hardware — regardless of the vendor
SDK underneath. It does *not* load or optimize models; that is the job of a
separate serving layer (see hiVLA). What hiRobot provides is the
embodiment-aware I/O contract (:class:`Observation` / :class:`Action`), the
:class:`Robot` driver interface, a driver registry, and the safety gate.
Running a policy on a robot (the control loop, the VLA client) is hiAgent's job.
"""

import logging
import os
from typing import Any, Mapping, Optional, Union

from hirobot.assembly import RobotAssembly
from hirobot.devices.audio import available_audio, get_audio, register_audio
from hirobot.devices.camera import available_cameras, get_camera, register_camera
from hirobot.core.dof import DofMap
from hirobot.core.embodiment import (
    AudioDevice,
    Camera,
    CameraMount,
    Embodiment,
    EmbodimentKind,
    EndEffector,
    EndEffectorFrame,
    Joint,
)
from hirobot.control.layer import Layer
from hirobot.devices.end_effector import available_end_effectors, get_end_effector, register_end_effector
from hirobot.loading.registry import (
    available_bodies,
    available_layers,
    get_body,
    get_layer,
    load_drivers,
    register_body,
    register_layer,
)
from hirobot.control.arbiter import CommandLease, PreemptedError, Priority, RobotBusyError
from hirobot.core.robot import Robot, RobotFaultError, RobotStoppedError
from hirobot.core.robot_skills import RobotSkillError, RobotSkillResult, RobotSkillSpec
from hirobot.control.safety import SafetyError, SafetyGate, SafetyVerdict
from hirobot.loading.sdk import require_sdk
from hirobot.core.types import (
    JOINT_MODES,
    MOTION_SPACES,
    Action,
    ActionChunk,
    BatteryState,
    CommandProgress,
    ControlMode,
    MotionClip,
    Observation,
    RobotStatus,
)

__version__ = "0.0.4"

__all__ = [
    "Observation",
    "Action",
    "ActionChunk",
    "ControlMode",
    "JOINT_MODES",
    "MOTION_SPACES",
    "MotionClip",
    "RobotStatus",
    "BatteryState",
    "CommandProgress",
    "Embodiment",
    "EmbodimentKind",
    "Joint",
    "EndEffector",
    "EndEffectorFrame",
    "Camera",
    "CameraMount",
    "AudioDevice",
    "DofMap",
    "Robot",
    "RobotAssembly",
    "Layer",
    "RobotFaultError",
    "RobotBusyError",
    "PreemptedError",
    "Priority",
    "CommandLease",
    "RobotStoppedError",
    "RobotSkillSpec",
    "RobotSkillResult",
    "RobotSkillError",
    "SafetyGate",
    "SafetyVerdict",
    "SafetyError",
    "register_body",
    "get_body",
    "available_bodies",
    "register_end_effector",
    "get_end_effector",
    "available_end_effectors",
    "register_camera",
    "get_camera",
    "available_cameras",
    "register_audio",
    "get_audio",
    "available_audio",
    "load_drivers",
    "register_layer",
    "get_layer",
    "available_layers",
    "require_sdk",
    "load_embodiment",
    "make_robot",
    "__version__",
]


def _sim_check(cfg: Any, spec: Any, driver_name: str) -> Any:
    """The ``safety.sim_check`` section as a :class:`~hirobot.sim.sim_check.SimCheck`.

    ``sim_check: true`` takes every default. The check world is built from
    the robot's own tree (``spec``): every body needs a ``model:``.
    """
    from hirobot.sim.sim_check import SimCheck

    cfg = {} if cfg is True else dict(cfg)
    if driver_name == "mujoco":
        raise ValueError("safety.sim_check predicts a command in MuJoCo; this robot already is "
                         "MuJoCo (the sim: simulation). Set it on the real robot's config")
    if cfg.get("model") is not None:
        raise ValueError("safety.sim_check.model is gone: the check builds its world from the "
                         "robot's tree (each body's model:); remove model:")
    if spec is None:
        raise ValueError("safety.sim_check needs an assembly: config (the old embodiment: "
                         "format has no model: per body)")
    from hirobot.loading.tree import DeviceSpec

    missing = [p for p, d in spec.root.walk()
               if isinstance(d, DeviceSpec) and d.kind == "body" and not d.fields.get("model")]
    if missing:
        raise ValueError(f"safety.sim_check: body part(s) {missing} have no model: (their MJCF); "
                         "every body needs one, checking only some parts would miss collisions")
    check = SimCheck.from_config(cfg)
    check.model = spec
    return check


def load_embodiment(config: Union[str, "os.PathLike[str]", dict]) -> "Embodiment":
    """The :class:`Embodiment` a robot config (``assembly:``) or device file describes.

    ``config`` is a built-in config name (``"unitree_g1"``), a file path, or an
    already-loaded dict; see :mod:`hirobot.loading.tree`. Nothing is connected or built.
    """
    from hirobot.loading.config import load_config
    from hirobot.loading.layout import node_layout
    from hirobot.loading.legacy import is_legacy_config, legacy_embodiment
    from hirobot.loading.tree import load_robot

    loaded = load_config(config)
    if is_legacy_config(loaded):  # deprecated, see hirobot.loading.legacy
        return legacy_embodiment(loaded)
    return node_layout(load_robot(loaded if isinstance(config, dict) else config).root).embodiment


def _warn_slow_cameras(embodiment: Embodiment) -> None:
    """Warn when the loop runs much faster than a camera's declared ``fps``."""
    for cam in embodiment.cameras:
        if cam.fps and embodiment.control_hz > 2 * cam.fps:
            logging.getLogger(__name__).warning(
                "camera %r delivers %.0f fps but control_hz is %d: the loop sees each frame "
                "about %.0f times (Observation.stamps[%r].seq tells repeats apart)",
                cam.name, cam.fps, embodiment.control_hz, embodiment.control_hz / cam.fps,
                cam.name)


def make_robot(
    config: Union[str, "os.PathLike[str]", dict],
    body_overrides: Optional[Mapping[str, Any]] = None,
    overrides: Optional[Mapping[str, Any]] = None,
    sim: Union[None, bool, Mapping[str, Any]] = None,
) -> "Robot":
    """Build the robot a config describes.

    ``config`` is a built-in config name (``"unitree_g1"``, ``"body/unitree/h1"``),
    a file path, or an already-loaded dict: a robot config (``assembly:``) or a
    device file alone (``device:``), loaded by :func:`hirobot.loading.tree.load_robot`
    and built as a tree by :mod:`hirobot.loading.build`: one robot per node, the drivers
    looked up in the registry (built-in drivers and ``hirobot.drivers``
    entry-point plugins).

    Deprecated, for code outside hiRobot that has not moved yet
    (:mod:`hirobot.loading.legacy`): a config in the old ``embodiment:`` + ``body:``
    format is still built, and ``body_overrides`` (old ``body:`` keys, with
    ``driver: <name>``) still applies, to that format or to the one body of an
    ``assembly:`` config. Both warn; ``group:`` configs are rejected.

    ``overrides`` maps a part path to overrides for that part, merged over the
    config files (``{"body": {"driver": {"network_interface": "eth0"}}}``,
    ``{"left/body": {"driver": {"ip": ...}}}``), so host-specific values can come
    from the command line instead of a shared config.

    A :class:`~hirobot.control.safety.SafetyGate` wraps the root, with per-joint limits
    from every ``safety:`` in the tree (the joint-count and non-finite checks
    always apply); ``safety: false`` detaches it. With a simulated body (the
    built-in robots default to ``toy``) only stand-in camera and audio drivers
    (``simulated = True``) are built, and a simulator models the fingers itself,
    so the config runs without the robot. With a real body, each hand with a
    driver gets an :class:`~hirobot.devices.end_effector.EndEffectorDriver` (read-only
    when the body is) and the body drives only its own joints.

    ``sim=True``, or a ``sim:`` section with ``sim`` left at ``None``, builds
    the robot in one shared MuJoCo world instead (:mod:`hirobot.sim.sim_world`):
    every body from its device file's ``model:``, at its placement, a view of
    the world as its driver; mounted cameras rendered by the world; no real
    driver built. ``sim=False`` builds the real robot of a config with ``sim:``.
    ``sim`` may also be a mapping, merged over the config's ``sim:`` section
    (``{"realtime": False, "viewer": True}``); it turns the simulation on.

    The stack, innermost first: the drivers; the
    :class:`~hirobot.assembly.RobotAssembly` joining bodies, end-effectors,
    cameras and audio (built only when there is a device to attach); the
    safety gate; then the optional ``layers:``. Each ``layers:`` entry is a
    mapping whose ``type`` names a registered layer
    (:func:`~hirobot.loading.registry.register_layer`, :class:`~hirobot.control.layer.Layer`),
    first entry innermost; the other keys go to it. A layer sees the whole
    robot, or with ``part: <path>`` only that part of an ``assembly:`` robot
    (:mod:`hirobot.assembly.part_layers`; these sit under the whole-robot layers). The
    gate under the layers checks the joint commands they send; with layers, a
    second gate on top checks what the caller sends.
    """
    from hirobot.loading.config import load_config
    from hirobot.loading.tree import is_tree_config

    from hirobot.loading.legacy import body_overrides_as_tree, is_legacy_config, make_legacy

    loaded = load_config(config)
    if is_legacy_config(loaded):
        if overrides or sim:
            raise ValueError("overrides= and sim= are for assembly: configs; this config is in "
                             "the old embodiment: format (use body_overrides)")
        return make_legacy(loaded, config, body_overrides)
    if not is_tree_config(loaded):
        old = [k for k in ("body", "robot", "group") if k in loaded]
        raise ValueError(
            f"{config!r} is not a robot config (assembly:) or a device file (device:)"
            + (f"; its {', '.join(k + ':' for k in old)} section(s) are the old format: "
               "write the robot as assembly: parts" if old else ""))
    if body_overrides:
        from hirobot.loading.config import merge

        overrides = merge(body_overrides_as_tree(config, body_overrides), dict(overrides or {}))
    return _make_tree(config, overrides, sim)


def _make_tree(ref: Any, overrides: Optional[Mapping[str, Any]], sim: Optional[bool]) -> Robot:
    """An ``assembly:`` / ``device:`` config built as a tree (:mod:`hirobot.loading.build`)."""
    from hirobot.loading.build import build_tree, tree_safety
    from hirobot.loading.tree import DeviceSpec, load_robot

    spec = load_robot(ref, overrides=overrides)
    in_sim = bool(sim) if sim is not None and not isinstance(sim, Mapping) \
        else isinstance(sim, Mapping) or spec.sim is not None
    if isinstance(sim, Mapping):
        from hirobot.loading.config import merge

        spec.sim = merge(dict(spec.sim or {}), dict(sim))
    robot, embodiment = build_tree(spec, sim=in_sim)
    _warn_slow_cameras(embodiment)
    safety_cfg: Any = False
    sim_cfg = None
    if spec.safety is not False:
        sim_cfg = (spec.safety or {}).get("sim_check")
        safety_cfg = tree_safety(spec, embodiment)
    bodies = [d for _, d in spec.root.walk() if isinstance(d, DeviceSpec) and d.kind == "body"]
    driver_name = ("mujoco" if in_sim else (bodies[0].driver or {}).get("type", "toy")
                   if len(bodies) == 1 else "assembly")
    return _finish(robot, embodiment, safety_cfg, sim_cfg, spec.layers, spec.arbiter, spec,
                   driver_name)


def _finish(robot: Robot, embodiment: Embodiment, safety_cfg: Any, sim_cfg: Any,
            layer_specs: Any, arbiter_cfg: Any, spec: Any, driver_name: str) -> Robot:
    """Gate, layers and arbiter around a built robot (both config formats)."""
    from hirobot.loading.registry import LAYERS

    # The hardware gate sits on the assembly, under every layer, so the joint
    # commands a layer generates clear it too.
    if safety_cfg is not False:
        gate = SafetyGate.from_embodiment(embodiment, **safety_cfg)
        if sim_cfg:
            gate.sim_check = _sim_check(sim_cfg, spec, driver_name)
            gate.sim_check.prepare(embodiment)
        robot._set_safety(gate)

    layer_specs = layer_specs or []
    layers = []
    if not isinstance(layer_specs, list):
        raise ValueError("layers: must be a list of mappings, each with a 'type'")
    whole, on_parts = [], []
    for spec in layer_specs:
        spec = dict(spec)
        if "type" not in spec:
            raise ValueError(f"layers: entry {spec!r} needs a 'type'")
        name = spec.pop("type")
        try:
            layer_cls = LAYERS.get(name)
        except KeyError as exc:
            raise KeyError(f"{exc.args[0]} A layer comes from a plugin package; "
                           "install the one that provides it.") from None
        if "part" in spec:
            part = spec.pop("part")
            if not isinstance(part, str):
                raise ValueError(f"layers: part: of {name!r} must be a part path, got {part!r}")
            on_parts.append((part.strip("/"), layer_cls, spec))
        else:
            whole.append((layer_cls, spec))
    if on_parts:
        # Layers on parts sit right above the hardware gate, under the
        # whole-robot layers (hirobot.assembly.part_layers).
        from hirobot.assembly.part_layers import PartLayers

        if getattr(robot, "_node", None) is None:
            raise ValueError("layers: part: is for a robot built from parts (an assembly: config "
                             "with child robots); this robot has none")
        robot = PartLayers(robot, on_parts)
        layers.append(robot)
        layers += list(robot.layers.values())
    for layer_cls, spec in whole:
        robot = layer_cls(robot, **spec)
        layers.append(robot)
    if layer_specs and safety_cfg is not False:
        # On top, the same checks see what the caller sends to the outermost
        # layer; the simulation check runs only on the hardware gate below.
        robot._set_safety(SafetyGate.from_embodiment(embodiment, **safety_cfg))

    # One arbiter on the outermost robot decides who may command the stack;
    # every layer gets a reference (a layer skill claims through it).
    from hirobot.control.arbiter import CommandArbiter

    arbiter_cfg = arbiter_cfg or {}
    if not isinstance(arbiter_cfg, dict) or set(arbiter_cfg) - {"require_claim", "audit_size"}:
        raise ValueError(f"arbiter: takes require_claim and audit_size, got {arbiter_cfg!r}")
    robot._arbiter = CommandArbiter(robot, require_claim=bool(arbiter_cfg.get("require_claim")),
                                    **({"audit_size": arbiter_cfg["audit_size"]}
                                       if "audit_size" in arbiter_cfg else {}))
    for layer in layers:
        layer._stack_arbiter = robot._arbiter
    return robot
