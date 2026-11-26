"""Declarative description of a robot's I/O contract.

An :class:`Embodiment` captures *what a robot looks like* to a model or agent —
its joints, whether it has a floating base, its end-effectors, cameras, and
control rate — independent of the hardware or driver used to talk to it. This is
the contract that lets the same policy target different humanoids and arms.

Joints are described **by name**, not just by count, so a policy trained against
one joint ordering can be mapped onto a robot with a different one (see the DoF
mapping helpers). The same :class:`Embodiment` type covers both a 7-DoF
fixed-base arm (Franka, UR) and a 30+ DoF humanoid (Unitree, Fourier, AgiBot):
the humanoid simply sets ``floating_base=True`` and lists more joints.
"""

from __future__ import annotations

import os
from enum import Enum
from typing import Any, Dict, List, Literal, Optional, Tuple

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class EmbodimentKind(str, Enum):
    """Broad class of robot, used to set sensible defaults and validation."""

    HUMANOID = "humanoid"
    ARM = "arm"


class Joint(BaseModel):
    """A single actuated joint.

    Attributes:
        name: Unique joint name; the key used for DoF mapping across robots.
        position_limit: ``(lower, upper)`` position bounds in rad, if known.
        torque_limit: Maximum absolute torque in Nm, if known.
        temperature_limit: ``(warn, shutdown)`` motor temperatures in degC, if
            known: ``warn`` is where a monitor should flag the joint as hot,
            ``shutdown`` where it is unsafe to keep commanding it. Both are
            positive and ``warn <= shutdown``. hiRobot does not refuse commands
            at ``warn``; it is the threshold a monitor (e.g. hiAPP) reads, while
            ``shutdown`` is the default the hardware gate's ``max_joint_temperature``
            falls back to.
        axis: Unit vector the joint turns about, in the frame of the link the
            joint moves (as URDF ``<axis>`` and MJCF ``axis``), if known.
        parent_link: Link the joint hangs from (URDF ``<parent>``), if known.
    """

    name: str
    position_limit: Optional[Tuple[float, float]] = None
    torque_limit: Optional[float] = None
    temperature_limit: Optional[Tuple[float, float]] = None
    axis: Optional[Tuple[float, float, float]] = None
    parent_link: Optional[str] = None

    @model_validator(mode="after")
    def _check(self) -> "Joint":
        if self.position_limit is not None and self.position_limit[0] > self.position_limit[1]:
            raise ValueError(f"joint {self.name!r}: position_limit {list(self.position_limit)} "
                             "has lower > upper")
        if self.torque_limit is not None and not self.torque_limit > 0:
            raise ValueError(f"joint {self.name!r}: torque_limit must be > 0, got {self.torque_limit}")
        if self.temperature_limit is not None:
            warn, shutdown = self.temperature_limit
            if not (warn > 0 and shutdown > 0):
                raise ValueError(f"joint {self.name!r}: temperature_limit "
                                 f"{list(self.temperature_limit)} must have both tiers > 0")
            if warn > shutdown:
                raise ValueError(f"joint {self.name!r}: temperature_limit "
                                 f"{list(self.temperature_limit)} has warn > shutdown; it is "
                                 "(warn, shutdown) in degC")
        if self.axis is not None and abs(sum(a * a for a in self.axis) ** 0.5 - 1.0) > 1e-3:
            raise ValueError(f"joint {self.name!r}: axis {list(self.axis)} is not a unit vector")
        return self


class CameraIntrinsics(BaseModel):
    """Pinhole intrinsics of a camera image, in pixels.

    Pixel ``(u, v)`` has its origin at the top-left corner of the image, ``u``
    to the right, ``v`` down, so pixel centres are at integer coordinates. A
    point ``(x, y, z)`` in the camera's optical frame (OpenCV: z forward, x
    right, y down; metres) projects to ``u = fx * x / z + cx``,
    ``v = fy * y / z + cy`` before distortion.

    Attributes:
        width: Image width these intrinsics belong to (pixels).
        height: Image height (pixels).
        fx: Focal length along ``u`` (pixels).
        fy: Focal length along ``v`` (pixels).
        cx: Principal point ``u`` (pixels).
        cy: Principal point ``v`` (pixels).
        model: Distortion model: ``none``, ``brown_conrady`` / ``plumb_bob``
            (OpenCV ``k1, k2, p1, p2, k3``), ``inverse_brown_conrady`` (RealSense),
            ``kannala_brandt`` (OpenCV fisheye ``k1..k4``).
        coeffs: The model's coefficients, in that order; empty for ``none``.
    """

    width: int = Field(gt=0)
    height: int = Field(gt=0)
    fx: float = Field(gt=0)
    fy: float = Field(gt=0)
    cx: float
    cy: float
    model: str = "none"
    coeffs: Tuple[float, ...] = ()

    @model_validator(mode="after")
    def _check_model(self) -> "CameraIntrinsics":
        if self.model not in DISTORTION_MODELS:
            raise ValueError(f"camera intrinsics: unknown distortion model {self.model!r}; "
                             f"use one of {list(DISTORTION_MODELS)}")
        n = DISTORTION_MODELS[self.model]
        if self.model == "none":
            if any(c != 0 for c in self.coeffs):
                raise ValueError("camera intrinsics: model 'none' takes no distortion coefficients")
            self.coeffs = ()
        elif len(self.coeffs) != n:
            raise ValueError(f"camera intrinsics: model {self.model!r} takes {n} coefficients, "
                             f"got {len(self.coeffs)}")
        return self

    @property
    def K(self) -> "Any":
        """The 3x3 camera matrix ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`` (numpy float64)."""
        import numpy as np

        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def scaled(self, width: int, height: int) -> "CameraIntrinsics":
        """The same camera at another resolution (a resize of the whole image, no crop)."""
        sx, sy = width / self.width, height / self.height
        return self.model_copy(update={
            "width": int(width), "height": int(height), "fx": self.fx * sx, "fy": self.fy * sy,
            "cx": (self.cx + 0.5) * sx - 0.5, "cy": (self.cy + 0.5) * sy - 0.5})

    @classmethod
    def from_fovy(cls, width: int, height: int, fovy_deg: float) -> "CameraIntrinsics":
        """An ideal pinhole with square pixels and the principal point at the centre.

        What a simulator renders for a vertical field of view of ``fovy_deg``.
        """
        import math

        f = (height / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
        return cls(width=width, height=height, fx=f, fy=f,
                   cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)


#: Distortion model name -> number of coefficients.
DISTORTION_MODELS: Dict[str, int] = {
    "none": 0,
    "brown_conrady": 5,
    "plumb_bob": 5,
    "inverse_brown_conrady": 5,
    "kannala_brandt": 4,
}


class CameraMount(BaseModel):
    """Where a camera sits on the robot, and its field of view.

    The same values describe the real camera (its extrinsics: the pose of its
    optical frame in a robot link, as calibrated) and place the rendered camera
    in a simulator, so both see from the same pose. A moving link (a head, a
    wrist) gives the pose in that link; composing it with the link's current
    pose is the caller's step.

    Attributes:
        body: Name of the robot link (MJCF body) the camera is fixed to.
        pos: Camera position in that link's frame (m).
        quat: Orientation ``[x, y, z, w]`` of the camera's optical frame in the
            link's frame. Optical frame as in OpenCV: z forward, x right, y down.
        fovy: Vertical field of view (degrees); the horizontal one follows from
            the image aspect ratio.
    """

    body: str
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    fovy: float = 45.0


class Camera(BaseModel):
    """A single camera stream exposed by an embodiment.

    Attributes:
        name: Key of the camera's frames in ``Observation.images``.
        width: Frame width in pixels, as delivered.
        height: Frame height in pixels, as delivered.
        driver: Optional camera-driver spec: ``type`` names a registered
            :class:`~hirobot.devices.camera.CameraDriver` and the remaining keys are
            its arguments. ``make_robot`` builds and attaches it. Leave it out
            for cameras the body driver renders itself (simulators).
        mount: Optional pose and field of view on the robot: the camera's
            extrinsics. A simulator body (``mujoco``) renders a camera with a
            mount at that pose; for a real camera it is the calibrated pose
            that callers read through ``Robot.camera_info``.
        intrinsics: Optional fixed intrinsics (a calibration). They win over
            what the driver reads from the camera; see ``Robot.camera_info``.
        depth: True for an RGB-D camera: its driver also delivers a depth
            image registered to the colour image, which appears in
            ``Observation.depth`` under the same name. One camera, not two, so
            the pair always comes from the same instant.
        fps: Nominal frame rate, if known. :func:`~hirobot.core.stamps.stale_sources`
            uses it to judge a frame stale, and ``make_robot`` warns when
            ``control_hz`` is far above it (the loop then sees each frame
            several times; ``Observation.stamps`` tells repeats apart).
    """

    name: str
    width: int
    height: int
    driver: Optional[Dict[str, Any]] = None
    mount: Optional[CameraMount] = None
    depth: bool = False
    fps: Optional[float] = None
    intrinsics: Optional[CameraIntrinsics] = None

    @model_validator(mode="before")
    @classmethod
    def _reject_source_key(cls, data: Any) -> Any:
        if isinstance(data, dict) and "source" in data:
            raise ValueError(f"camera {data.get('name')!r}: the source: key is now called "
                             "driver:; rename it")
        return data


class CameraInfo(BaseModel):
    """What a caller needs to turn a camera's pixels and depth into 3D: see ``Robot.camera_info``.

    Attributes:
        name: The camera (its key in ``Observation.images``).
        intrinsics: Pinhole intrinsics of ``images[name]`` (and of
            ``depth[name]``, which is registered to it), or ``None`` when
            neither the config nor the driver gives them and there is no
            ``fovy`` to derive them from.
        intrinsics_source: Where they came from: ``config`` (the camera's
            ``intrinsics:``), ``driver`` (read from the camera after
            ``connect()``), ``fovy`` (an ideal pinhole from the mount's field of
            view and the image size, what a simulator renders), or ``None``.
        mount: The pose of the camera's optical frame in a robot link (the
            extrinsics), or ``None`` when the config does not place the camera.
        depth: Whether the camera delivers ``Observation.depth[name]`` (metres,
            registered to the colour image, 0 where there is no reading).
    """

    name: str
    intrinsics: Optional[CameraIntrinsics] = None
    intrinsics_source: Optional[str] = None
    mount: Optional[CameraMount] = None
    depth: bool = False


class EndEffectorFrame(BaseModel):
    """Which link of the robot model an end-effector's pose is read from.

    Attributes:
        body: Name of the link (MJCF body), e.g. ``left_wrist_yaw_link``.
        pos: Origin of the end-effector frame in that link's frame (m).
        quat: Orientation ``[x, y, z, w]`` of the end-effector frame in the link's frame.
    """

    body: str
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


class EndEffector(BaseModel):
    """A controllable end-effector / task-space frame.

    Attributes:
        name: End-effector name; the key used in task-space observations and
            actions (``ee_poses`` / ``ee_targets``).
        has_gripper: Whether the end-effector has a gripper commanded through
            ``Action.gripper`` (and read in ``Observation.gripper``).
        joints: Names of the embodiment joints this end-effector actuates (a
            dexterous hand's finger joints), in its driver's motor order. They
            are part of ``Embodiment.joints`` like any other joint.
        driver: Optional end-effector driver spec: ``type`` names a registered
            :class:`~hirobot.devices.end_effector.EndEffectorDriver` and the remaining
            keys are its arguments. With ``joints`` the driver uses its
            ``joints`` interface; without, its ``opening`` interface, which
            needs ``has_gripper: true``. With a real body driver, ``make_robot``
            builds it and routes the end-effector's joints or gripper to it; a
            simulator drives those joints itself and the spec is ignored.
        frame: Optional link of the robot model the pose is read from. The
            ``mujoco`` body reports ``ee_poses`` for every end-effector with a
            frame; drivers whose SDK reports the pose themselves (UR, Franka)
            ignore it. :class:`~hirobot.geometry.kinematics.Kinematics` computes the
            same pose from joint angles.
    """

    name: str
    has_gripper: bool = False
    joints: List[str] = Field(default_factory=list)
    driver: Optional[Dict[str, Any]] = None
    frame: Optional[EndEffectorFrame] = None


class AudioDevice(BaseModel):
    """A microphone or speaker mounted on the robot.

    Attributes:
        name: The device's key in ``robot.microphones`` / ``robot.speakers``.
        driver: Audio-driver spec: ``type`` names a registered
            :class:`~hirobot.drivers.audio.MicrophoneDriver` or
            :class:`~hirobot.drivers.audio.SpeakerDriver` and the remaining keys
            are its arguments. ``make_robot`` builds and attaches it; with a
            simulator only drivers marked ``simulated`` (WAV files, the host's
            own microphone and speaker) are built and the rest are skipped.
    """

    name: str
    driver: Optional[Dict[str, Any]] = None


class ModelRef(BaseModel):
    """The model file of a robot's body (``Embodiment.model``), for drawing or simulating it.

    Frozen and hashable. Fields added later are optional, so code that reads these
    keeps working.

    Attributes:
        kind: ``"mjcf"`` or ``"urdf"``: the format of ``path``. A body file whose
            ``model:`` has both gives the MJCF.
        path: Absolute path of the file, ``~`` expanded. Whether the file exists
            is not checked; meshes are found the way the format finds them (MJCF
            ``meshdir``, URDF ``package://`` through ``package_roots``).
        package_roots: URDF only: package name -> absolute directory, to resolve
            ``package://<name>/...`` mesh paths. Empty for MJCF.
    """

    model_config = ConfigDict(frozen=True)

    kind: Literal["mjcf", "urdf"]
    path: str
    package_roots: Dict[str, str] = Field(default_factory=dict)

    @field_validator("path")
    @classmethod
    def _absolute(cls, value: str) -> str:
        path = os.path.expanduser(value)
        if not os.path.isabs(path):
            raise ValueError(f"model path {value!r} must be absolute")
        return path

    @field_validator("package_roots")
    @classmethod
    def _absolute_roots(cls, value: Dict[str, str]) -> Dict[str, str]:
        out = {str(k): os.path.expanduser(str(v)) for k, v in value.items()}
        relative = sorted(k for k, v in out.items() if not os.path.isabs(v))
        if relative:
            raise ValueError(f"package_roots {relative} must be absolute paths")
        return out

    @model_validator(mode="after")
    def _roots_for_urdf(self) -> "ModelRef":
        if self.package_roots and self.kind != "urdf":
            raise ValueError("package_roots is for a URDF model only")
        return self

    def __hash__(self) -> int:
        # pydantic's frozen hash fails on the dict field; hash its sorted items instead.
        return hash((self.kind, self.path, tuple(sorted(self.package_roots.items()))))


class EmbodimentPart(BaseModel):
    """One part of a robot built from an ``assembly:`` config (``Embodiment.parts``).

    Attributes:
        joints: ``(start, stop)`` of the part's joints in the robot's joint
            vector (``joint_pos[start:stop]``).
        end_effectors: Its end effectors, by their names in the robot.
        cameras: Its cameras, by their names in the robot.
        microphones: Its microphones, by their names in the robot.
        speakers: Its speakers, by their names in the robot.
        children: Paths of its child parts that are robots of their own (nodes).
        moving: Whether the part moves by itself (a floating base, or a node
            whose frame moves with one).
        devices: The ``parts:`` keys of its own devices (body, hands, cameras,
            audio), not of its child parts.
    """

    joints: Tuple[int, int] = (0, 0)
    end_effectors: List[str] = Field(default_factory=list)
    cameras: List[str] = Field(default_factory=list)
    microphones: List[str] = Field(default_factory=list)
    speakers: List[str] = Field(default_factory=list)
    children: List[str] = Field(default_factory=list)
    moving: bool = False
    devices: List[str] = Field(default_factory=list)


class Embodiment(BaseModel):
    """The I/O contract of a robot, independent of its driver.

    Attributes:
        name: Human-readable embodiment identifier.
        kind: Whether this is a humanoid or an arm.
        control_hz: Target control frequency the loop should run at.
        joints: The actuated joints, in the canonical order the driver uses for
            ``joint_pos`` / ``joint_target`` vectors.
        floating_base: True for robots with a moving base (humanoids) whose
            observations include IMU / base state; False for fixed-base arms.
        end_effectors: Task-space frames available for Cartesian control.
        cameras: Camera streams the robot exposes.
        microphones: Microphones on the robot.
        speakers: Speakers on the robot.
        parts: For a robot built from an ``assembly:`` config: part path ->
            :class:`EmbodimentPart`. ``""`` is the whole robot; a child node
            ``left`` of the root is ``"left"``, its child ``"left/arm"``. Only
            nodes are listed (not single devices). Empty for an embodiment
            written by hand.
        battery_soc_limit: ``(warn, shutdown)`` battery state of charge in
            percent, if known. The battery is one pack for the whole robot, so
            it lives here, not on a joint. ``warn`` is where a monitor should
            flag the charge as low, ``shutdown`` where it is unsafe to keep
            commanding; the warning fires at a *higher* charge than shutdown, so
            ``warn >= shutdown`` and both are within 0-100. hiRobot does not
            refuse commands at ``warn``; it is the threshold a monitor (e.g.
            hiAPP) reads, while ``shutdown`` is the default the hardware gate's
            ``min_battery_soc`` falls back to.
        model: The model file of the robot's one body (:class:`ModelRef`), or
            ``None`` when the robot has more than one body or its body file has
            no ``model:``. Filled from the config by ``load_embodiment`` and
            ``make_robot``, for real robots too. Nothing is opened.
    """

    name: str
    kind: EmbodimentKind = EmbodimentKind.HUMANOID
    control_hz: int = 30
    joints: List[Joint] = Field(default_factory=list)
    floating_base: bool = True
    end_effectors: List[EndEffector] = Field(default_factory=list)
    cameras: List[Camera] = Field(default_factory=list)
    microphones: List[AudioDevice] = Field(default_factory=list)
    speakers: List[AudioDevice] = Field(default_factory=list)
    parts: Dict[str, EmbodimentPart] = Field(default_factory=dict)
    battery_soc_limit: Optional[Tuple[float, float]] = None
    model: Optional[ModelRef] = None

    @model_validator(mode="after")
    def _check_battery_soc_limit(self) -> "Embodiment":
        if self.battery_soc_limit is not None:
            warn, shutdown = self.battery_soc_limit
            if not (warn > 0 and shutdown > 0):
                raise ValueError(f"embodiment {self.name!r}: battery_soc_limit "
                                 f"{list(self.battery_soc_limit)} must have both tiers > 0")
            if not (0 <= warn <= 100 and 0 <= shutdown <= 100):
                raise ValueError(f"embodiment {self.name!r}: battery_soc_limit "
                                 f"{list(self.battery_soc_limit)} must be within 0-100 percent")
            if warn < shutdown:
                raise ValueError(f"embodiment {self.name!r}: battery_soc_limit "
                                 f"{list(self.battery_soc_limit)} has warn < shutdown; it is "
                                 "(warn, shutdown) in percent, warning at a higher charge")
        return self

    @model_validator(mode="after")
    def _check_end_effector_joints(self) -> "Embodiment":
        names = set(self.joint_names)
        if len(names) != len(self.joints):
            raise ValueError(f"embodiment {self.name!r} has duplicate joint names")
        claimed: Dict[str, str] = {}
        for ee in self.end_effectors:
            for j in ee.joints:
                if j not in names:
                    raise ValueError(f"end-effector {ee.name!r} lists unknown joint {j!r}")
                if j in claimed:
                    raise ValueError(f"joint {j!r} is listed by both {claimed[j]!r} and {ee.name!r}")
                claimed[j] = ee.name
            if ee.driver is not None and ee.joints and ee.has_gripper:
                raise ValueError(
                    f"end-effector {ee.name!r}: a driver with joints uses its 'joints' "
                    "interface, so the gripper is commanded through joint_target; drop "
                    "has_gripper, or drop joints to use the 'opening' interface")
            if ee.driver is not None and not ee.joints and not ee.has_gripper:
                raise ValueError(
                    f"end-effector {ee.name!r} has a driver but neither joints (a hand, "
                    "'joints' interface) nor has_gripper: true (a gripper, 'opening' interface)")
        sources = ["body"] + [ee.name for ee in self.end_effectors] + [c.name for c in self.cameras]
        dup = sorted({s for s in sources if sources.count(s) > 1})
        if dup:
            raise ValueError(f"embodiment {self.name!r}: {dup} name more than one of the body "
                             "('body'), an end-effector and a camera; Observation.stamps keys "
                             "them by name, so the names must differ")
        for kind, devices in (("microphone", self.microphones), ("speaker", self.speakers)):
            seen = [d.name for d in devices]
            if len(set(seen)) != len(seen):
                raise ValueError(f"embodiment {self.name!r} has duplicate {kind} names")
        return self

    @property
    def joint_names(self) -> List[str]:
        """Canonical joint names, in the order used by I/O vectors."""
        return [j.name for j in self.joints]

    @property
    def num_dofs(self) -> int:
        """Number of actuated joints."""
        return len(self.joints)
