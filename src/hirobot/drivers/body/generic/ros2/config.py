"""The ``driver:`` section of the ``ros2`` body driver, checked without rclpy.

Standard messages are components: each ``type`` names one message class and
the config only gives the topic. A custom command message is ``type: message``
with ``msg: pkg/msg/Name`` and a ``fields`` map from message fields to sources
(``$joint_target`` ...) or constants. Everything that can be checked without
the message classes is checked here, when the driver is built; the field names
themselves are checked in ``connect()`` once the class is loaded.
"""

from __future__ import annotations

from typing import Annotated, Any, Dict, List, Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from hirobot.core.types import ControlMode

#: Sources a ``message`` command's ``fields`` may name.
SOURCES = ("$joint_names", "$joint_target", "$kp", "$kd", "$stamp")

#: Modes each command component can carry.
JOINT_MODES = (ControlMode.POSITION, ControlMode.VELOCITY, ControlMode.TORQUE, ControlMode.PD)


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class QoS(_Strict):
    """Reliability and queue depth of one subscription or publisher."""

    reliability: Literal["reliable", "best_effort"]
    depth: int = Field(ge=1)


class _Component(_Strict):
    topic: str
    qos: Optional[QoS] = None

    @field_validator("topic")
    @classmethod
    def _topic(cls, v: str) -> str:
        if not v or " " in v:
            raise ValueError(f"bad topic {v!r}")
        return v


# ------------------------------------------------------------------ state


class JointStateConfig(_Component):
    """``sensor_msgs/msg/JointState``: joint_pos / joint_vel / joint_torque by name.

    ``joints`` limits the component to some joints (embodiment names), for a
    robot that publishes several JointState topics; default all of them.
    """

    type: Literal["joint_state"]
    joints: Optional[List[str]] = None


class ImuConfig(_Component):
    """``sensor_msgs/msg/Imu``: base orientation, angular velocity, linear acceleration."""

    type: Literal["imu"]


class OdometryConfig(_Component):
    """``nav_msgs/msg/Odometry``: base position and linear velocity."""

    type: Literal["odometry"]


StateConfig = Annotated[Union[JointStateConfig, ImuConfig, OdometryConfig],
                        Field(discriminator="type")]


# ------------------------------------------------------------------ command


class JointTrajectoryConfig(_Component):
    """``trajectory_msgs/msg/JointTrajectory`` with one point: POSITION targets by name."""

    type: Literal["joint_trajectory"]
    time_from_start_s: float = Field(default=0.0, ge=0.0)

    @property
    def modes(self) -> List[ControlMode]:
        return [ControlMode.POSITION]


class FloatArrayConfig(_Component):
    """``std_msgs/msg/Float64MultiArray``: one mode, by index in ``joints`` order.

    ros2_control's forward_command_controller takes its joints in the order of
    its own ``joints:`` parameter; ``joints`` here lists the same order
    (embodiment names; default the embodiment's order).
    """

    type: Literal["float_array"]
    mode: Literal["position", "velocity", "torque"]
    joints: Optional[List[str]] = None

    @property
    def modes(self) -> List[ControlMode]:
        return [ControlMode(self.mode)]


class MessageConfig(_Component):
    """Any command message, filled from ``fields``.

    Keys are message fields (dots for nested ones, ``header.frame_id``); values
    are a source from :data:`SOURCES` or a constant. Fields not listed keep the
    message's defaults.
    """

    type: Literal["message"]
    msg: str
    modes: List[ControlMode]
    fields: Dict[str, Any]

    @field_validator("msg")
    @classmethod
    def _msg(cls, v: str) -> str:
        parts = v.split("/")
        if len(parts) == 2:
            parts = [parts[0], "msg", parts[1]]
        if len(parts) != 3 or parts[1] != "msg" or not all(parts):
            raise ValueError(f"msg {v!r} must look like 'pkg/msg/Name'")
        return v

    @model_validator(mode="after")
    def _check(self) -> "MessageConfig":
        if not self.modes:
            raise ValueError("modes is empty")
        bad = [m.value for m in self.modes if m not in JOINT_MODES]
        if bad:
            raise ValueError(f"modes {bad} not supported; use {[m.value for m in JOINT_MODES]}")
        if not self.fields:
            raise ValueError("fields is empty")
        for key, value in self.fields.items():
            if not key or any(not p.isidentifier() for p in key.split(".")):
                raise ValueError(f"bad field name {key!r}")
            if isinstance(value, str) and value.startswith("$") and value not in SOURCES:
                raise ValueError(f"fields.{key}: unknown source {value!r}; use one of {SOURCES}")
        used = {v for v in self.fields.values() if isinstance(v, str)}
        if "$joint_target" not in used:
            raise ValueError("fields must map some field to $joint_target")
        if used & {"$kp", "$kd"} and ControlMode.PD not in self.modes:
            raise ValueError("fields uses $kp / $kd but modes has no 'pd'")
        return self


CommandConfig = Annotated[Union[JointTrajectoryConfig, FloatArrayConfig, MessageConfig],
                          Field(discriminator="type")]


# ------------------------------------------------------------------ driver


class Ros2Config(_Strict):
    """Everything under ``driver:`` except ``type``."""

    node_name: str = "hirobot_ros2"
    ros_names: Dict[str, str] = Field(default_factory=dict)
    connect_timeout_s: float = Field(default=5.0, gt=0.0)
    read_only: bool = False
    default_kp: Optional[float] = None
    default_kd: Optional[float] = None
    state: List[StateConfig]
    command: List[CommandConfig] = Field(default_factory=list)

    @field_validator("state", mode="before")
    @classmethod
    def _state_types(cls, v: Any) -> Any:
        return _check_types(v, "state", ("joint_state", "imu", "odometry"))

    @field_validator("command", mode="before")
    @classmethod
    def _command_types(cls, v: Any) -> Any:
        return _check_types(v, "command", ("joint_trajectory", "float_array", "message"))

    @model_validator(mode="after")
    def _check(self) -> "Ros2Config":
        if not any(isinstance(s, JointStateConfig) for s in self.state):
            raise ValueError("state needs a joint_state component")
        seen: Dict[ControlMode, int] = {}
        for i, cmd in enumerate(self.command):
            for mode in cmd.modes:
                if mode in seen:
                    raise ValueError(
                        f"mode {mode.value!r} is in command[{seen[mode]}] and command[{i}]; "
                        "each mode needs one component")
                seen[mode] = i
            fields = getattr(cmd, "fields", {})
            for src, default in (("$kp", self.default_kp), ("$kd", self.default_kd)):
                if src in fields.values() and default is None and cmd.modes != [ControlMode.PD]:
                    raise ValueError(
                        f"command[{i}] fills a field from {src} in modes other than pd, where "
                        f"the action carries no gains; set default_{src[1:]}")
        for kind in ("imu", "odometry"):
            if sum(s.type == kind for s in self.state) > 1:
                raise ValueError(f"state has more than one {kind} component")
        return self

    @property
    def modes(self) -> List[ControlMode]:
        """Every mode some command component carries, in config order."""
        return [m for cmd in self.command for m in cmd.modes]

    def command_for(self, mode: ControlMode) -> Optional[CommandConfig]:
        for cmd in self.command:
            if mode in cmd.modes:
                return cmd
        return None


def _check_types(items: Any, section: str, known: tuple) -> Any:
    """A clear error for a missing or unknown ``type`` (pydantic's union error is long)."""
    if not isinstance(items, list):
        return items
    for i, item in enumerate(items):
        t = item.get("type") if isinstance(item, dict) else None
        if t not in known:
            raise ValueError(f"{section}[{i}].type is {t!r}; use one of {list(known)}")
    return items
