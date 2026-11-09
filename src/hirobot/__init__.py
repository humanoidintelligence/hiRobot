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
from typing import Any, Optional, Union

from hirobot.assembly import RobotAssembly
from hirobot.audio import available_audio, get_audio, register_audio
from hirobot.camera import available_cameras, get_camera, register_camera
from hirobot.dof import DofMap
from hirobot.embodiment import (
    AudioDevice,
    Camera,
    CameraMount,
    Embodiment,
    EmbodimentKind,
    EndEffector,
    EndEffectorFrame,
    Joint,
)
from hirobot.group import RobotGroup
from hirobot.end_effector import available_end_effectors, get_end_effector, register_end_effector
from hirobot.registry import (
    available_bodies,
    available_layers,
    get_body,
    get_layer,
    load_drivers,
    register_body,
    register_layer,
)
from hirobot.robot import Robot, RobotFaultError, RobotStoppedError
from hirobot.robot_skills import RobotSkillError, RobotSkillResult, RobotSkillSpec
from hirobot.safety import SafetyError, SafetyGate, SafetyVerdict
from hirobot.sdk import require_sdk
from hirobot.types import (
    JOINT_MODES,
    MOTION_SPACES,
    Action,
    ActionChunk,
    BatteryState,
    ControlMode,
    MotionClip,
    Observation,
    RobotStatus,
)

__version__ = "0.0.1"

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
    "RobotGroup",
    "RobotFaultError",
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


def _embodiment_of(config: dict, ref: Any) -> "Embodiment":
    embodiment = config.get("embodiment")
    if embodiment is None:
        raise ValueError(f"No 'embodiment' section found in {ref!r}.")
    return Embodiment(**embodiment)


def load_embodiment(config: Union[str, "os.PathLike[str]", dict]) -> "Embodiment":
    """Load an :class:`Embodiment` from the ``embodiment`` section of a config.

    ``config`` is a built-in config name (``"unitree_g1"``), a file path, or an
    already-loaded dict; see :mod:`hirobot.config`.
    """
    from hirobot.config import load_config

    return _embodiment_of(load_config(config), config)


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
    body_overrides: Optional[dict] = None,
    sim_overrides: Optional[dict] = None,
) -> "Robot":
    """Construct the configured :class:`Robot` driver from a config.

    ``config`` is a built-in config name (``"unitree_g1"``, ``"body/unitree/h1"``),
    a file path, or an already-loaded dict. ``defaults:`` lists are expanded
    first (see :mod:`hirobot.config`).

    Reads ``embodiment`` and the ``body.driver`` name, looks the driver up in
    the registry, and instantiates it with the embodiment. The registry imports
    the built-in drivers and any ``hirobot.drivers`` entry-point plugins on first use.

    ``body_overrides`` is merged over the config's ``body:`` section, so
    host-specific values (network interface, robot IP, ...) can come from the
    command line instead of being written into a shared config.

    A :class:`~hirobot.safety.SafetyGate` is attached from the embodiment (so the
    joint-count and non-finite checks always apply) and tuned by the optional
    ``safety:`` section (``joint_limit``, ``clamp``, ``gripper_range``). Every
    action then clears the gate inside ``send_action``. Set ``safety: false`` to
    detach it.

    The optional ``sim:`` section pairs the robot with a MuJoCo twin (see
    :mod:`hirobot.twin`): ``mode: off`` (or no section) drives the robot
    directly and needs no MuJoCo install; ``mirror`` renders the real robot's
    state; ``shadow`` opens the real robot read-only and sends every action to
    the simulation instead. ``sim_overrides`` is merged over that section
    (e.g. ``{"mode": "shadow"}`` from a command-line flag).

    Cameras declared with a ``driver:`` in the embodiment get a camera driver
    each, attached through :class:`~hirobot.assembly.RobotAssembly`. With a
    simulated body (the shipped ``toy`` default) only stand-in camera drivers
    whose class sets ``simulated = True`` (``video_file``, ``array``) are built;
    real cameras are skipped so the config still runs without the robot.

    Microphones and speakers with a ``driver:`` are attached the same way, with
    the same rule (``wav_mic``, ``wav_speaker``, ``null_speaker`` and the host's
    ``sounddevice_*`` count as simulated), and reached as ``robot.microphones`` /
    ``robot.speakers``.

    End-effectors with a ``driver:`` (dexterous hands, grippers) are handled the
    same way. With a real driver, the body driver gets the embodiment minus the
    end-effectors' joints, one :class:`~hirobot.end_effector.EndEffectorDriver` is
    built per end-effector (read-only when the body is), and the assembly presents
    all joints as one vector. A simulated driver models the fingers itself and gets every joint.

    A config with a ``group:`` section (and no ``body:`` / ``embodiment:``)
    builds a :class:`~hirobot.group.RobotGroup` instead: each member is a
    whole single-robot config built by this function, and ``body_overrides``
    is keyed by member name (``{"right": {"ip": ...}}``). See
    :func:`hirobot.group.make_group`.

    The optional ``layers:`` list wraps the robot in registered layers (see
    :func:`~hirobot.registry.register_layer`), first entry innermost, after the
    sim twin and before cameras and audio are attached. Each entry is a mapping
    whose ``type`` names the layer; the other keys go to it. Layers come from
    plugin packages, hiRobot ships none. A layer sees every joint: with real
    hands, the body and hands are joined first, so a layer can drive the
    fingers itself or leave them to the caller. The safety gate is attached
    both under the layers (to the joint commands they send) and on top (to
    what the caller sends).
    """
    from hirobot.config import load_config
    from hirobot.end_effector import (
        body_embodiment,
        build_end_effectors,
        has_end_effector_drivers,
    )
    from hirobot.registry import LAYERS

    ref = config
    config = load_config(config)
    if "group" in config:
        from hirobot.config import resolve
        from hirobot.group import make_group

        base_dir = None if isinstance(ref, dict) else str(resolve(ref).parent)
        return make_group(config, base_dir, body_overrides, sim_overrides)
    if "robot" in config:
        raise ValueError("the config's robot: section is now called body:; rename it")
    embodiment = _embodiment_of(config, ref)
    _warn_slow_cameras(embodiment)
    body_cfg = dict(config.get("body") or {})
    body_cfg.update(body_overrides or {})
    sim_cfg = dict(config.get("sim") or {})
    sim_cfg.update(sim_overrides or {})
    sim_mode = sim_cfg.pop("mode", "off") if sim_cfg else "off"
    if sim_mode is False or sim_mode is None:  # YAML 1.1 reads a bare `off` as False
        sim_mode = "off"
    if sim_mode not in ("off", "mirror", "shadow"):
        raise ValueError(f"sim.mode must be off / mirror / shadow, got {sim_mode!r}")
    driver_name = body_cfg.pop("driver", "toy")
    if sim_mode == "shadow":
        if body_cfg.get("release_motion_mode"):
            raise ValueError("sim mode 'shadow' never commands the real robot; drop release_motion_mode")
        body_cfg["read_only"] = True
        # Built-in skills command the real robot, so a shadow run offers none.
        body_cfg.pop("builtin_skills", None)
    driver_cls = get_body(driver_name)
    # A real body drives only its own joints; end-effector drivers take the
    # finger joints. A simulator models the fingers itself and gets every joint.
    end_effectors = {}
    body_emb = embodiment
    if not driver_cls.simulated and has_end_effector_drivers(embodiment):
        end_effectors = build_end_effectors(embodiment, body_cfg.get("network_interface"),
                                            read_only=bool(body_cfg.get("read_only")))
        body_emb = body_embodiment(embodiment)
    # Remaining body.* keys are driver-specific connection params (network
    # interface, server IP, robot IP, ...), passed through to the driver.
    robot = driver_cls(body_emb, **body_cfg)

    if sim_mode != "off":
        from hirobot.twin import TwinRobot

        if "model" not in sim_cfg:
            raise ValueError("sim: section needs 'model' (path to the MJCF file)")
        sim_cfg["model_path"] = sim_cfg.pop("model")
        # The twin uses the same gains as the real robot unless overridden.
        for key in ("default_kp", "default_kd", "ramp_seconds", "damping_kd"):
            if key in body_cfg and key not in sim_cfg:
                sim_cfg[key] = body_cfg[key]
        mirror_hz = sim_cfg.pop("mirror_hz", 50.0)
        if end_effectors:
            # The real side is body + end_effectors (all joints) so the twin, which
            # models the fingers, mirrors and shadows the whole robot.
            robot = RobotAssembly(robot, end_effectors=end_effectors, embodiment=embodiment)
            end_effectors = {}
        # The twin renders no cameras: the images come from the real ones.
        twin_emb = embodiment.model_copy(update={"cameras": [
            c.model_copy(update={"mount": None}) for c in embodiment.cameras]})
        twin = get_body("mujoco")(twin_emb, **sim_cfg)
        robot = TwinRobot(robot, twin, mode=sim_mode, mirror_hz=mirror_hz)

    layer_specs = config.get("layers") or []
    if not isinstance(layer_specs, list):
        raise ValueError("layers: must be a list of mappings, each with a 'type'")
    safety_cfg = config.get("safety", {})
    if layer_specs:
        if end_effectors:
            robot = RobotAssembly(robot, end_effectors=end_effectors, embodiment=embodiment)
            end_effectors = {}
        if safety_cfg is not False:
            robot.set_safety(SafetyGate.from_embodiment(embodiment, **(safety_cfg or {})))
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
            robot = layer_cls(robot, **spec)

    # A simulated body gets only stand-in devices (video file, WAV, the host's
    # own microphone); real hardware is skipped so the config runs offline.
    from hirobot.audio import build_audio
    from hirobot.camera import build_cameras

    sim_only = bool(driver_cls.simulated)
    nic = body_cfg.get("network_interface")
    cameras = build_cameras(embodiment.cameras, nic, sim_only=sim_only)
    microphones, speakers = build_audio(embodiment.microphones, embodiment.speakers, nic,
                                        sim_only=sim_only)
    if cameras or end_effectors or microphones or speakers:
        robot = RobotAssembly(robot, cameras=cameras, end_effectors=end_effectors,
                              embodiment=embodiment, microphones=microphones, speakers=speakers)

    if safety_cfg is not False:
        robot.set_safety(SafetyGate.from_embodiment(embodiment, **(safety_cfg or {})))
    return robot
