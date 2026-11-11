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

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from pydantic import BaseModel, Field, model_validator


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
    """

    name: str
    position_limit: Optional[Tuple[float, float]] = None
    torque_limit: Optional[float] = None


class CameraMount(BaseModel):
    """Where a camera sits on the robot, and its field of view.

    The same values describe the real camera (its extrinsics) and place the
    rendered camera in a simulator, so both see from the same pose.

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
        mount: Optional pose and field of view on the robot. A simulator body
            (``mujoco``) renders a camera with a mount at that pose; the real
            camera's driver ignores it.
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

    @model_validator(mode="before")
    @classmethod
    def _reject_source_key(cls, data: Any) -> Any:
        if isinstance(data, dict) and "source" in data:
            raise ValueError(f"camera {data.get('name')!r}: the source: key is now called "
                             "driver:; rename it")
        return data


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
