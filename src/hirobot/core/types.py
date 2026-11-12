"""Typed I/O schemas shared across robots, models, and agents.

These schemas describe what a policy or agent consumes (:class:`Observation`)
and produces (:class:`Action`) in an embodiment-aware but driver-agnostic way,
so the same control code can target different humanoids and arms, in simulation
or on real hardware.

The design is layered so one schema spans very different embodiments:

* **Joint space** is the universal core — every robot we target (Unitree,
  Fourier GRx, AgiBot X1, Franka, UR) exposes joint positions/velocities and
  accepts position/velocity/torque commands with PD gains.
* **Floating base** fields (IMU, base pose/twist) are optional: humanoids
  populate them, fixed-base arms leave them ``None``.
* **Task space** fields (end-effector poses) are optional: arms and dexterous
  humanoids use them, locomotion-only stacks leave them empty.
* **Perception** fields (images, instruction) support VLA models.

Everything optional defaults to empty/``None`` so a driver only fills what its
robot actually provides, and a consumer reads only what it needs.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from hirobot.core.stamps import Stamp


class ControlMode(str, Enum):
    """How a driver should interpret the values in an :class:`Action`.

    Names map onto the modes every surveyed SDK exposes in some form:
    position/velocity/torque joint control, PD targets with per-command gains,
    and Cartesian (task-space) control for arms.
    """

    POSITION = "position"      # joint position targets
    VELOCITY = "velocity"      # joint velocity targets
    TORQUE = "torque"          # joint torque/effort targets
    PD = "pd"                  # position targets tracked by (kp, kd) gains
    CARTESIAN = "cartesian"    # end-effector pose/velocity targets (arms)
    # Per-tick end-effector deltas [dx, dy, dz, dax, day, daz] (axis-angle) in
    # ee_targets, as robosuite's OSC_POSE controller takes them.
    CARTESIAN_DELTA = "cartesian_delta"
    # Commands for a robot that balances itself (a whole-body controller): the
    # controller turns them into joint targets every tick. hiRobot only defines
    # them; a layer or driver that can follow one lists it in action_modes.
    # BASE_VELOCITY: [vx, vy, wz] in the body frame (m/s, m/s, rad/s) in
    # base_velocity. Not VELOCITY, which is per-joint velocity.
    BASE_VELOCITY = "base_velocity"
    # MOTION_LATENT: an opaque motion token (e.g. a SONIC latent) in latent.
    MOTION_LATENT = "motion_latent"
    # MOTION_CLIP: a reference motion to track, a MotionClip in motion.
    MOTION_CLIP = "motion_clip"
    # BASE_POSE: walk to [x, y, yaw] in base_target, in the frame of
    # Observation.base_position; the robot stands still once it arrives.
    BASE_POSE = "base_pose"


#: Modes whose ``joint_target`` is one value per joint. The others command the
#: robot some other way and may still carry joint targets for a subset of joints
#: (see :attr:`Action.joint_names`).
JOINT_MODES = (ControlMode.POSITION, ControlMode.VELOCITY, ControlMode.TORQUE, ControlMode.PD)

#: Kinds of :class:`MotionClip`: ``joint`` is the robot's own joint space (by
#: joint name), ``smpl`` a human motion as SMPL joint positions.
MOTION_SPACES = ("joint", "smpl")


def _frames(value: Any, name: str, tail: tuple) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 1 + len(tail) or any(t is not None and arr.shape[1 + i] != t
                                         for i, t in enumerate(tail)):
        want = ", ".join("n" if t is None else str(t) for t in tail)
        raise ValueError(f"{name} must have shape (T, {want}), got {arr.shape}")
    return arr


class MotionClip(BaseModel):
    """A reference motion for a robot to track (a whole-body controller, usually).

    The clip is plain data, independent of any controller: a motion generator
    (a text-to-motion model, a VLA that outputs motion, a mocap file) produces it
    and whatever follows it turns it into its own input. Frames are evenly spaced
    at ``fps``.

    Attributes:
        fps: Frame rate (frames per second).
        space: One of :data:`MOTION_SPACES`. ``joint``: ``joint_names`` and
            ``joint_pos`` (and optionally ``joint_vel``) in the robot's joint
            space, matched by name, so a clip may cover only some joints.
            ``smpl``: ``smpl_joints``, human joint positions in the root frame.
        joint_names: Joint names for the columns of ``joint_pos`` / ``joint_vel``.
        joint_pos: ``(T, n)`` joint positions (rad).
        joint_vel: ``(T, n)`` joint velocities (rad/s); optional.
        smpl_joints: ``(T, J, 3)`` SMPL joint positions (m), root frame.
        root_pos: ``(T, 3)`` root position (m); optional.
        root_quat: ``(T, 4)`` root orientation ``[x, y, z, w]``; optional.
        extra: Free-form additional data.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    fps: float
    space: str = "joint"
    joint_names: Optional[List[str]] = None
    joint_pos: Optional[np.ndarray] = None
    joint_vel: Optional[np.ndarray] = None
    smpl_joints: Optional[np.ndarray] = None
    root_pos: Optional[np.ndarray] = None
    root_quat: Optional[np.ndarray] = None
    extra: Dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _check(self) -> "MotionClip":
        if not self.fps > 0:
            raise ValueError(f"fps must be positive, got {self.fps}")
        if self.space not in MOTION_SPACES:
            raise ValueError(f"space must be one of {MOTION_SPACES}, got {self.space!r}")
        if self.space == "joint":
            if self.joint_names is None or self.joint_pos is None:
                raise ValueError("a 'joint' clip needs joint_names and joint_pos")
            if len(set(self.joint_names)) != len(self.joint_names):
                raise ValueError("joint_names has duplicates")
            n = len(self.joint_names)
            self.joint_pos = _frames(self.joint_pos, "joint_pos", (n,))
            if self.joint_vel is not None:
                self.joint_vel = _frames(self.joint_vel, "joint_vel", (n,))
        else:
            if self.smpl_joints is None:
                raise ValueError("an 'smpl' clip needs smpl_joints")
            self.smpl_joints = _frames(self.smpl_joints, "smpl_joints", (None, 3))
        if self.root_pos is not None:
            self.root_pos = _frames(self.root_pos, "root_pos", (3,))
        if self.root_quat is not None:
            self.root_quat = _frames(self.root_quat, "root_quat", (4,))
        lengths = {name: arr.shape[0] for name, arr in self.arrays().items()}
        if len(set(lengths.values())) != 1 or not next(iter(lengths.values())):
            raise ValueError(f"every array needs the same number of frames T >= 1, got {lengths}")
        return self

    def arrays(self) -> Dict[str, np.ndarray]:
        """The per-frame arrays that are set, by field name."""
        names = ("joint_pos", "joint_vel", "smpl_joints", "root_pos", "root_quat")
        return {n: getattr(self, n) for n in names if getattr(self, n) is not None}

    @property
    def num_frames(self) -> int:
        """Number of frames ``T``."""
        return int(next(iter(self.arrays().values())).shape[0])

    @property
    def duration_s(self) -> float:
        """Clip length in seconds (``num_frames / fps``)."""
        return self.num_frames / self.fps


class Observation(BaseModel):
    """A single timestep observation from a robot.

    Only ``joint_pos`` / ``joint_vel`` are expected to be present for every
    embodiment. Base and end-effector fields are populated by robots that have
    them (humanoids and arms respectively); perception fields by VLA setups.

    Attributes:
        joint_pos: Joint positions (rad), ordered to match the embodiment.
        joint_vel: Joint velocities (rad/s).
        joint_torque: Measured/estimated joint torques (Nm), if available.
        base_orientation: Floating-base orientation quaternion ``[x, y, z, w]``
            (scalar last). Drivers whose SDK reports ``[w, x, y, z]`` reorder it.
        base_angular_velocity: Base angular velocity (rad/s), typically from IMU.
        base_linear_acceleration: Base linear acceleration (m/s^2) from IMU.
        base_position: Base position in world [x, y, z], if known (usually sim).
        base_linear_velocity: Base linear velocity (m/s), if known.
        ee_poses: End-effector name -> 7-vector [x, y, z, qx, qy, qz, qw].
        gripper: End-effector name -> measured gripper opening in [0, 1],
            0 closed and 1 open (the same scale as ``Action.gripper``).
        images: Camera name -> HxWxC uint8 array.
        depth: Camera name -> HxW float32 depth in metres, registered to that
            camera's image (same size, same pixel grid); 0 where the sensor has
            no reading. Only cameras declared with ``depth: true`` fill it.
        stamps: Source name -> :class:`~hirobot.core.stamps.Stamp` (arrival time on
            the robot's clock, :meth:`~hirobot.core.robot.Robot.now`, and reading
            number) of the reading this snapshot holds. Sources are ``body``,
            each end-effector with its own driver (a hand), and each camera; a
            robot group prefixes them (``left/body``). A source the snapshot has
            no reading from is absent. See :mod:`hirobot.core.stamps`.
        instruction: Optional natural-language task instruction.
        extra: Free-form additional signals a model may consume.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    # Joint space (universal).
    joint_pos: Optional[np.ndarray] = None
    joint_vel: Optional[np.ndarray] = None
    joint_torque: Optional[np.ndarray] = None

    # Floating base (humanoids; None for fixed-base arms).
    base_orientation: Optional[np.ndarray] = None
    base_angular_velocity: Optional[np.ndarray] = None
    base_linear_acceleration: Optional[np.ndarray] = None
    base_position: Optional[np.ndarray] = None
    base_linear_velocity: Optional[np.ndarray] = None

    # Task space (arms / dexterous end-effectors).
    ee_poses: Dict[str, np.ndarray] = Field(default_factory=dict)
    gripper: Dict[str, float] = Field(default_factory=dict)

    # Perception (VLA).
    images: Dict[str, np.ndarray] = Field(default_factory=dict)
    depth: Dict[str, np.ndarray] = Field(default_factory=dict)
    instruction: Optional[str] = None

    stamps: Dict[str, Stamp] = Field(default_factory=dict)
    extra: Dict[str, Any] = Field(default_factory=dict)


#: Action fields that command something (all but ``extra`` and ``parts``).
_ACTION_COMMAND_FIELDS = ("joint_target", "kp", "kd", "joint_names", "ee_targets", "gripper",
                          "base_velocity", "base_target", "latent", "motion", "append")
_CHUNK_COMMAND_FIELDS = ("joint_targets", "kp", "kd", "joint_names", "ee_targets", "gripper",
                         "base_velocity", "base_target", "latent")


def _is_set(value: Any) -> bool:
    return value is not None and value is not False and not (isinstance(value, dict) and not value)


class Action(BaseModel):
    """A command produced by a policy for a robot to execute.

    The meaning of the fields is governed by ``mode``. For joint modes
    (:data:`JOINT_MODES`: ``POSITION`` / ``VELOCITY`` / ``TORQUE`` / ``PD``),
    ``joint_target`` holds one value per joint; ``PD`` additionally uses
    ``kp`` / ``kd``. For ``CARTESIAN``, ``ee_targets`` holds per-end-effector
    goals and joint fields may be left empty; for ``CARTESIAN_DELTA`` it holds
    per-tick deltas.

    The other modes command a robot that moves itself (a whole-body controller):
    ``BASE_VELOCITY`` (``base_velocity``), ``BASE_POSE`` (``base_target``),
    ``MOTION_LATENT`` (``latent``), ``MOTION_CLIP`` (``motion``). ``CARTESIAN``
    may carry ``base_velocity`` too (reach while walking). Any of these may also
    carry position targets for some joints the controller leaves free (fingers,
    or arms under a legs-only controller): ``joint_names`` names them and
    ``joint_target`` / ``kp`` / ``kd`` hold one value per name. Which joints a
    robot accepts this way is :attr:`~hirobot.core.robot.Robot.direct_joints`.

    Attributes:
        mode: How a driver should interpret this action.
        joint_target: Per-joint target (position/velocity/torque per ``mode``);
            with ``joint_names``, position targets for those joints only.
        kp: Per-joint proportional gains (``PD`` mode, or with ``joint_names``).
        kd: Per-joint derivative gains (``PD`` mode, or with ``joint_names``).
        joint_names: Names of the joints ``joint_target`` covers, for the
            non-joint modes only; ``None`` means every joint in embodiment order.
        ee_targets: End-effector name -> 7-vector pose target (``CARTESIAN``),
            or 6-vector ``[dx, dy, dz, dax, day, daz]`` delta (``CARTESIAN_DELTA``).
        gripper: End-effector name -> gripper opening target in [0, 1], 0
            closed and 1 open. Drivers convert to their hardware's own range
            and direction.
        base_velocity: ``[vx, vy, wz]`` body-frame base velocity (``BASE_VELOCITY``,
            or alongside ``CARTESIAN``).
        base_target: ``[x, y, yaw]`` to walk to (``BASE_POSE``), in the frame of
            ``Observation.base_position``.
        latent: Opaque motion token (``MOTION_LATENT``).
        motion: Reference motion to track (``MOTION_CLIP``).
        append: With ``MOTION_CLIP``, queue ``motion`` after the clip already
            playing instead of replacing it (streamed motion generation).
        extra: Free-form additional outputs (e.g. value estimates, logits).
        parts: For a robot with parts (an ``assembly:`` config): part name ->
            that part's action, in the part's own names (no prefix), each with
            its own mode. A part's action may have ``parts`` of its own (a
            node inside a node). When set, every other field but ``extra``
            must be left empty. A robot without parts refuses it.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    mode: ControlMode = ControlMode.POSITION
    joint_target: Optional[np.ndarray] = None
    kp: Optional[np.ndarray] = None
    kd: Optional[np.ndarray] = None
    joint_names: Optional[List[str]] = None
    ee_targets: Dict[str, np.ndarray] = Field(default_factory=dict)
    gripper: Dict[str, float] = Field(default_factory=dict)
    base_velocity: Optional[np.ndarray] = None
    base_target: Optional[np.ndarray] = None
    latent: Optional[np.ndarray] = None
    motion: Optional[MotionClip] = None
    append: bool = False
    extra: Dict[str, Any] = Field(default_factory=dict)
    parts: Dict[str, "Action"] = Field(default_factory=dict)

    @model_validator(mode="before")
    @classmethod
    def _renamed(cls, data: Any) -> Any:
        if isinstance(data, dict) and "members" in data:
            raise ValueError("Action.members is now called Action.parts")
        return data

    @model_validator(mode="after")
    def _check(self) -> "Action":
        if self.parts:
            used = [f for f in _ACTION_COMMAND_FIELDS if _is_set(getattr(self, f))]
            if used:
                raise ValueError(f"an Action with parts takes no other command fields; got {used}")
            return self
        if self.joint_names is not None:
            if self.mode in JOINT_MODES:
                raise ValueError(
                    f"joint_names is for the non-joint modes; {self.mode.value!r} takes "
                    "one joint_target value per joint")
            if len(set(self.joint_names)) != len(self.joint_names):
                raise ValueError("joint_names has duplicates")
            if self.joint_target is None:
                raise ValueError("joint_names needs joint_target")
        if self.mode == ControlMode.MOTION_CLIP and self.motion is None:
            raise ValueError("MOTION_CLIP needs motion")
        if self.mode == ControlMode.BASE_POSE and self.base_target is None:
            raise ValueError("BASE_POSE needs base_target")
        return self


class ActionChunk(BaseModel):
    """A sequence of future actions predicted in one shot.

    Many VLA policies (e.g. AgiBot's GO-1, which predicts 30-step chunks) emit a
    horizon of actions per inference call rather than one step at a time. This
    schema holds that horizon; a driver still executes one :class:`Action` per
    control tick, so a chunk-aware policy caches a chunk and replays it with
    :meth:`step`. Keeping chunks separate from :class:`Action` leaves single-step
    drivers unchanged.

    The leading axis of ``joint_targets`` / each ``ee_targets`` entry is the
    horizon ``T``. ``kp`` / ``kd`` are shared across the chunk (one gain per
    joint), matching how these policies emit position targets with fixed gains.

    Attributes:
        mode: How every step in the chunk should be interpreted.
        joint_targets: Per-step joint targets, shape ``(T, num_joints)``.
        kp: Per-joint proportional gains shared across the chunk (``PD`` mode).
        kd: Per-joint derivative gains shared across the chunk (``PD`` mode).
        ee_targets: End-effector name -> per-step poses, shape ``(T, 7)``
            (``(T, 6)`` deltas for ``CARTESIAN_DELTA``).
        gripper: Gripper name -> per-step command, shape ``(T,)``. Manipulation
            policies emit gripper open/close alongside arm motion, so the chunk
            carries it per step (vs the shared ``kp`` / ``kd`` gains).
        joint_names: The joints ``joint_targets`` covers, for the non-joint
            modes (see :class:`Action`); ``None`` means every joint.
        base_velocity: Per-step ``[vx, vy, wz]``, shape ``(T, 3)`` (``BASE_VELOCITY``).
        base_target: Per-step ``[x, y, yaw]``, shape ``(T, 3)`` (``BASE_POSE``).
        latent: Per-step motion tokens, shape ``(T, D)`` (``MOTION_LATENT``).
        extra: Free-form additional outputs.
        parts: For a robot with parts: part name -> that part's chunk (own
            names, own mode, may nest). Every part chunk has the same horizon;
            the other fields but ``extra`` stay empty. ``step(i)`` gives an
            :class:`Action` with ``parts``.

    A :class:`MotionClip` is already a sequence, so it travels as one
    ``Action(mode=MOTION_CLIP)``, not as a chunk.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    mode: ControlMode = ControlMode.POSITION
    joint_targets: Optional[np.ndarray] = None
    kp: Optional[np.ndarray] = None
    kd: Optional[np.ndarray] = None
    joint_names: Optional[List[str]] = None
    ee_targets: Dict[str, np.ndarray] = Field(default_factory=dict)
    gripper: Dict[str, np.ndarray] = Field(default_factory=dict)
    base_velocity: Optional[np.ndarray] = None
    base_target: Optional[np.ndarray] = None
    latent: Optional[np.ndarray] = None
    extra: Dict[str, Any] = Field(default_factory=dict)
    parts: Dict[str, "ActionChunk"] = Field(default_factory=dict)

    @model_validator(mode="before")
    @classmethod
    def _renamed(cls, data: Any) -> Any:
        if isinstance(data, dict) and "members" in data:
            raise ValueError("ActionChunk.members is now called ActionChunk.parts")
        return data

    @model_validator(mode="after")
    def _check_parts(self) -> "ActionChunk":
        if self.parts:
            used = [f for f in _CHUNK_COMMAND_FIELDS if _is_set(getattr(self, f))]
            if used:
                raise ValueError(f"an ActionChunk with parts takes no other command fields; "
                                 f"got {used}")
            horizons = {m: c.horizon for m, c in self.parts.items()}
            if len(set(horizons.values())) > 1:
                raise ValueError(f"part chunks have different horizons: {horizons}")
        return self

    @property
    def horizon(self) -> int:
        """Number of steps ``T`` in the chunk."""
        for chunk in self.parts.values():
            return chunk.horizon
        for arr in (self.joint_targets, self.base_velocity, self.base_target, self.latent):
            if arr is not None:
                return int(np.asarray(arr).shape[0])
        for poses in self.ee_targets.values():
            return int(np.asarray(poses).shape[0])
        for cmd in self.gripper.values():
            return int(np.asarray(cmd).shape[0])
        return 0

    def __len__(self) -> int:
        return self.horizon

    def step(self, i: int) -> Action:
        """Extract the ``i``-th step as a single-step :class:`Action`.

        Raises:
            IndexError: If ``i`` is outside ``[0, horizon)``.
        """
        if not 0 <= i < self.horizon:
            raise IndexError(f"step {i} out of range for horizon {self.horizon}")
        if self.parts:
            return Action(parts={m: c.step(i) for m, c in self.parts.items()},
                          extra=self.extra)
        return Action(
            mode=self.mode,
            joint_target=None if self.joint_targets is None else self.joint_targets[i],
            kp=self.kp,
            kd=self.kd,
            joint_names=self.joint_names,
            ee_targets={name: np.asarray(poses)[i] for name, poses in self.ee_targets.items()},
            gripper={name: float(np.asarray(cmd)[i]) for name, cmd in self.gripper.items()},
            base_velocity=None if self.base_velocity is None else np.asarray(self.base_velocity)[i],
            base_target=None if self.base_target is None else np.asarray(self.base_target)[i],
            latent=None if self.latent is None else np.asarray(self.latent)[i],
            extra=self.extra,
        )


class BatteryState(BaseModel):
    """Battery pack readings, in SI units. Fields a robot does not report stay ``None``.

    Attributes:
        soc: State of charge, percent (0-100).
        voltage: Pack voltage (V).
        current: Pack current (A); negative while discharging.
        temperature: Hottest pack temperature sensor (degC).
        health: State of health, percent (0-100).
        cycles: Charge cycle count.
    """

    soc: Optional[float] = None
    voltage: Optional[float] = None
    current: Optional[float] = None
    temperature: Optional[float] = None
    health: Optional[float] = None
    cycles: Optional[int] = None


class CommandProgress(BaseModel):
    """How far the robot is with the last high-level command it was given.

    For commands a controller follows over many ticks (``BASE_POSE``,
    ``MOTION_CLIP``, ``BASE_VELOCITY``), so a caller (an agent's tool) can tell
    whether the robot arrived or the clip ended without knowing the driver.
    The controller keeps the last one after it ends (``done=True``) until the
    next command replaces it. Fields a mode does not use stay ``None``.

    Attributes:
        mode: The command's mode.
        done: ``True`` once a ``BASE_POSE`` goal is reached or a ``MOTION_CLIP``
            has played to its end; ``None`` for a mode with no end
            (``BASE_VELOCITY``).
        goal: ``BASE_POSE`` target ``[x, y, yaw]``, in the frame of
            :attr:`Observation.base_position`.
        clip_time: Seconds of the ``MOTION_CLIP`` played so far.
        clip_duration: Length of the ``MOTION_CLIP`` (seconds); ``None`` if
            unknown.
    """

    mode: ControlMode
    done: Optional[bool] = None
    goal: Optional[List[float]] = None
    clip_time: Optional[float] = None
    clip_duration: Optional[float] = None


class RobotStatus(BaseModel):
    """Robot health and housekeeping: battery, temperatures, motor faults.

    Kept out of :class:`Observation` on purpose: a remote policy forwards every
    observation field to the model server each tick, and these signals are for
    the operating layer (an agent deciding whether to start a long task, a UI),
    not the model. Read it with :meth:`~hirobot.core.robot.Robot.get_status`, at
    whatever rate the caller needs.

    Attributes:
        timestamp: Host time (``time.time()``) the status was assembled.
        battery: Battery readings, or ``None`` if the robot reports none.
        joint_temperature: Per-joint motor temperature (degC), in embodiment
            order; the hottest sensor where a motor has several.
        joint_faults: Joint name -> raw vendor fault code, nonzero codes only;
            empty means no joint reports a fault.
        board_temperature: Board name -> temperature (degC), e.g. ``mainboard``.
        fault: Why the robot refuses commands (it damped or stopped itself),
            or ``None``; see :attr:`~hirobot.core.robot.Robot.fault`.
        extra: Vendor-specific readings that have no core field (cell voltages,
            fan states, ...).
        safety: Counters of the safety gate's optional checks, e.g.
            ``safety["sim_check"]`` (``checked``, ``rejected``, ``errors``,
            ``skipped``, ``failures_in_row``, ``p99_ms``, ``last_reason``).
        command: Progress of the last high-level command (:class:`CommandProgress`),
            or ``None`` when the robot reports none (no command yet, or a
            driver that does not track one).
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    timestamp: Optional[float] = None
    battery: Optional[BatteryState] = None
    joint_temperature: Optional[np.ndarray] = None
    joint_faults: Dict[str, int] = Field(default_factory=dict)
    board_temperature: Dict[str, float] = Field(default_factory=dict)
    fault: Optional[str] = None
    extra: Dict[str, Any] = Field(default_factory=dict)
    safety: Dict[str, Any] = Field(default_factory=dict)
    command: Optional[CommandProgress] = None
