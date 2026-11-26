"""The ``ros2`` driver's components: one class per message type.

A state component subscribes to one topic and keeps the latest message (the
callback runs on the shared executor thread, so under a lock); ``read(obs)``
fills its fields of an :class:`~hirobot.core.types.Observation`. A command
component publishes one topic; ``publish(action, kp, kd)`` builds and sends
the message. Joints are matched by name: each component gets the embodiment's
joint names and their ROS names (``ros_names``).
"""

from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from hirobot.core.stamps import Stamp, StampCounter
from hirobot.core.types import Action, Observation
from hirobot.drivers._sdk.ros2 import load_msg
from hirobot.drivers.body.generic.ros2.config import (
    FloatArrayConfig,
    ImuConfig,
    JointStateConfig,
    JointTrajectoryConfig,
    MessageConfig,
    OdometryConfig,
)

WHO = "ros2"

STATE_QOS = ("best_effort", 1)
COMMAND_QOS = ("reliable", 10)


def make_qos(qos: Any, default: tuple) -> Any:
    """An rclpy ``QoSProfile`` from a component's ``qos`` (or the default)."""
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    reliability, depth = (qos.reliability, qos.depth) if qos is not None else default
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliability == "reliable"
                     else ReliabilityPolicy.BEST_EFFORT))


# ------------------------------------------------------------------ state


class StateComponent:
    """Subscribes to one topic and keeps its latest message."""

    msg_type = ""  # pkg/msg/Name

    def __init__(self, config: Any) -> None:
        self.config = config
        self.topic = config.topic
        self._lock = threading.Lock()
        self._msg: Any = None
        self._stamps = StampCounter()
        self._stamp: Optional[Stamp] = None
        self._sub: Any = None

    def connect(self, node: Any) -> None:
        cls = load_msg(self.msg_type, WHO)
        self._sub = node.create_subscription(cls, self.topic, self._on_msg,
                                             make_qos(self.config.qos, STATE_QOS))

    def _on_msg(self, msg: Any) -> None:
        with self._lock:
            self._msg = msg
            self._stamp = self._stamps.tick()

    def latest(self) -> tuple:
        """``(message, stamp)``; both ``None`` before the first message."""
        with self._lock:
            return self._msg, self._stamp

    def read(self, obs: Observation) -> None:  # pragma: no cover - abstract
        raise NotImplementedError


class JointStateComponent(StateComponent):
    """``sensor_msgs/msg/JointState`` -> joint_pos / joint_vel / joint_torque."""

    msg_type = "sensor_msgs/msg/JointState"

    def __init__(self, config: JointStateConfig, joint_names: Sequence[str],
                 ros_names: Dict[str, str]) -> None:
        super().__init__(config)
        mine = config.joints or list(joint_names)
        unknown = [j for j in mine if j not in joint_names]
        if unknown:
            raise ValueError(f"{WHO}: joint_state {self.topic} lists joints not in the "
                             f"embodiment: {unknown}")
        #: (index in the embodiment, ROS name) for each joint this topic carries.
        self.joints = [(list(joint_names).index(j), ros_names.get(j, j)) for j in mine]
        self._order: Optional[tuple] = None  # (msg.name tuple, indices)

    def missing(self) -> List[str]:
        """ROS names this component should carry that its latest message does not."""
        msg, _ = self.latest()
        names = set(msg.name) if msg is not None else set()
        return [ros for _, ros in self.joints if ros not in names]

    def _indices(self, names: Sequence[str]) -> List[int]:
        key = tuple(names)
        if self._order is None or self._order[0] != key:
            where = {n: i for i, n in enumerate(names)}
            missing = [ros for _, ros in self.joints if ros not in where]
            if missing:
                raise RuntimeError(f"{WHO}: {self.topic} has no joints {missing}")
            self._order = (key, [where[ros] for _, ros in self.joints])
        return self._order[1]

    def read_into(self, pos: np.ndarray, vel: np.ndarray, eff: np.ndarray) -> tuple:
        """Copy this topic's joints into the full vectors; return (has_vel, has_eff)."""
        msg, _ = self.latest()
        idx = self._indices(msg.name)
        dst = [i for i, _ in self.joints]
        pos[dst] = np.asarray(msg.position, dtype=float)[idx]
        has_vel = len(msg.velocity) == len(msg.name)
        has_eff = len(msg.effort) == len(msg.name)
        if has_vel:
            vel[dst] = np.asarray(msg.velocity, dtype=float)[idx]
        if has_eff:
            eff[dst] = np.asarray(msg.effort, dtype=float)[idx]
        return has_vel, has_eff


class ImuComponent(StateComponent):
    """``sensor_msgs/msg/Imu`` -> base orientation ([x, y, z, w]), angular velocity, acceleration."""

    msg_type = "sensor_msgs/msg/Imu"

    def __init__(self, config: ImuConfig) -> None:
        super().__init__(config)

    def read(self, obs: Observation) -> None:
        msg, _ = self.latest()
        if msg is None:
            return
        o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        obs.base_orientation = np.array([o.x, o.y, o.z, o.w], dtype=float)
        obs.base_angular_velocity = np.array([w.x, w.y, w.z], dtype=float)
        obs.base_linear_acceleration = np.array([a.x, a.y, a.z], dtype=float)


class OdometryComponent(StateComponent):
    """``nav_msgs/msg/Odometry`` -> base position and linear velocity."""

    msg_type = "nav_msgs/msg/Odometry"

    def __init__(self, config: OdometryConfig) -> None:
        super().__init__(config)

    def read(self, obs: Observation) -> None:
        msg, _ = self.latest()
        if msg is None:
            return
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        obs.base_position = np.array([p.x, p.y, p.z], dtype=float)
        obs.base_linear_velocity = np.array([v.x, v.y, v.z], dtype=float)


# ------------------------------------------------------------------ command


class CommandComponent:
    """Publishes one topic."""

    msg_type = ""

    def __init__(self, config: Any, joint_names: Sequence[str], ros_names: Dict[str, str]) -> None:
        self.config = config
        self.topic = config.topic
        self.modes = list(config.modes)
        self.ros_joint_names = [ros_names.get(j, j) for j in joint_names]
        self._cls: Any = None
        self._pub: Any = None
        self._node: Any = None

    def connect(self, node: Any) -> None:
        self._node = node
        self._cls = load_msg(self._type_name(), WHO)
        self._check(self._cls)
        self._pub = node.create_publisher(self._cls, self.topic,
                                          make_qos(self.config.qos, COMMAND_QOS))

    def disconnect(self) -> None:
        self._pub = self._node = None

    def _type_name(self) -> str:
        return self.msg_type

    def _check(self, cls: Any) -> None:
        """Raise if the message class lacks what this component fills."""

    def publish(self, action: Action, kp: np.ndarray, kd: np.ndarray) -> None:
        self._pub.publish(self.build(action, kp, kd))

    def build(self, action: Action, kp: np.ndarray, kd: np.ndarray) -> Any:  # pragma: no cover
        raise NotImplementedError


class JointTrajectoryComponent(CommandComponent):
    """``trajectory_msgs/msg/JointTrajectory``: one point of position targets."""

    msg_type = "trajectory_msgs/msg/JointTrajectory"

    def __init__(self, config: JointTrajectoryConfig, joint_names, ros_names) -> None:
        super().__init__(config, joint_names, ros_names)
        self._point_cls: Any = None
        self._duration_cls: Any = None

    def connect(self, node: Any) -> None:
        super().connect(node)
        self._point_cls = load_msg("trajectory_msgs/msg/JointTrajectoryPoint", WHO)
        self._duration_cls = load_msg("builtin_interfaces/msg/Duration", WHO)

    def build(self, action: Action, kp: np.ndarray, kd: np.ndarray) -> Any:
        msg = self._cls()
        msg.joint_names = list(self.ros_joint_names)
        point = self._point_cls()
        point.positions = [float(x) for x in action.joint_target]
        t = self.config.time_from_start_s
        dur = self._duration_cls()
        dur.sec = int(t)
        dur.nanosec = int(round((t - int(t)) * 1e9))
        point.time_from_start = dur
        msg.points = [point]
        return msg


class FloatArrayComponent(CommandComponent):
    """``std_msgs/msg/Float64MultiArray``: one mode's targets, in ``joints`` order."""

    msg_type = "std_msgs/msg/Float64MultiArray"

    def __init__(self, config: FloatArrayConfig, joint_names, ros_names) -> None:
        super().__init__(config, joint_names, ros_names)
        order = config.joints or list(joint_names)
        unknown = [j for j in order if j not in joint_names]
        if unknown:
            raise ValueError(f"{WHO}: float_array {self.topic} lists joints not in the "
                             f"embodiment: {unknown}")
        if sorted(order) != sorted(joint_names):
            raise ValueError(f"{WHO}: float_array {self.topic} joints must list every joint "
                             f"once; missing {sorted(set(joint_names) - set(order))}")
        self._index = [list(joint_names).index(j) for j in order]

    def build(self, action: Action, kp: np.ndarray, kd: np.ndarray) -> Any:
        msg = self._cls()
        target = np.asarray(action.joint_target, dtype=float)
        msg.data = [float(x) for x in target[self._index]]
        return msg


class MessageComponent(CommandComponent):
    """Any message, filled from the config's ``fields``."""

    def __init__(self, config: MessageConfig, joint_names, ros_names) -> None:
        super().__init__(config, joint_names, ros_names)
        self.fields = dict(config.fields)

    def _type_name(self) -> str:
        return self.config.msg

    def _check(self, cls: Any) -> None:
        bad = []
        for key in self.fields:
            msg_cls, ok = cls, True
            for part in key.split("."):
                types = _field_types(msg_cls)
                if types is not None and part not in types:
                    ok = False
                    break
                msg_cls = _nested_class(msg_cls, part, types)
                if msg_cls is None:
                    break
            if not ok:
                bad.append(key)
        if bad:
            known = sorted(_field_types(cls) or ())
            raise ValueError(f"{WHO}: {self.config.msg} has no fields {bad}; it has {known}")

    def build(self, action: Action, kp: np.ndarray, kd: np.ndarray) -> Any:
        msg = self._cls()
        for key, value in self.fields.items():
            _set(msg, key, self._value(value, action, kp, kd))
        return msg

    def _value(self, value: Any, action: Action, kp: np.ndarray, kd: np.ndarray) -> Any:
        if not (isinstance(value, str) and value.startswith("$")):
            return list(value) if isinstance(value, (list, tuple)) else value
        if value == "$joint_names":
            return list(self.ros_joint_names)
        if value == "$joint_target":
            return [float(x) for x in action.joint_target]
        if value == "$kp":
            return [float(x) for x in kp]
        if value == "$kd":
            return [float(x) for x in kd]
        if value == "$stamp":
            return self._node.get_clock().now().to_msg()
        raise ValueError(f"{WHO}: unknown source {value!r}")  # pragma: no cover - config checks


def _field_types(cls: Any) -> Optional[Dict[str, str]]:
    """``{field: type}`` of a generated ROS 2 message class, or ``None`` if it cannot tell."""
    get = getattr(cls, "get_fields_and_field_types", None)
    return dict(get()) if callable(get) else None


def _nested_class(cls: Any, field: str, types: Optional[Dict[str, str]]) -> Any:
    """The message class of a nested field (``header`` -> ``std_msgs/Header``), if known."""
    if types is None or field not in types:
        return None
    t = types[field]
    if "/" not in t or t.startswith("sequence") or "[" in t:
        return None
    try:
        return load_msg(t, WHO)
    except (ImportError, ValueError):
        return None


def _set(msg: Any, key: str, value: Any) -> None:
    *path, last = key.split(".")
    for part in path:
        msg = getattr(msg, part)
    setattr(msg, last, value)


def make_state(config: Any, joint_names: Sequence[str], ros_names: Dict[str, str]):
    if isinstance(config, JointStateConfig):
        return JointStateComponent(config, joint_names, ros_names)
    if isinstance(config, ImuConfig):
        return ImuComponent(config)
    return OdometryComponent(config)


def make_command(config: Any, joint_names: Sequence[str], ros_names: Dict[str, str]):
    cls = {JointTrajectoryConfig: JointTrajectoryComponent, FloatArrayConfig: FloatArrayComponent,
           MessageConfig: MessageComponent}[type(config)]
    return cls(config, joint_names, ros_names)
