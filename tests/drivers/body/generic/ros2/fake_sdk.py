"""Fake ROS 2 for the ``ros2`` driver: rclpy, an executor, and the message packages it uses.

Messages are small classes with ``get_fields_and_field_types`` like the
generated ones, so the driver's field checks run. Nothing is spun in the
background: ``Ros.publish_state`` calls the subscriptions' callbacks at once,
as the real executor thread would shortly after a message arrives.
"""

from __future__ import annotations

import sys
import types
from typing import Any, Dict, List


class Msg:
    """A message with ROS-style fields; ``_fields`` maps name -> (ROS type, default factory)."""

    _fields: Dict[str, tuple] = {}

    def __init__(self, **kw: Any) -> None:
        for name, (_t, default) in self._fields.items():
            setattr(self, name, default())
        for k, v in kw.items():
            setattr(self, k, v)

    @classmethod
    def get_fields_and_field_types(cls) -> Dict[str, str]:
        return {k: t for k, (t, _d) in cls._fields.items()}


def _msg(cls_name: str, /, **fields: tuple) -> type:
    return type(cls_name, (Msg,), {"_fields": fields})


Time = _msg("Time", sec=("int32", int), nanosec=("uint32", int))
Duration = _msg("Duration", sec=("int32", int), nanosec=("uint32", int))
Header = _msg("Header", stamp=("builtin_interfaces/Time", Time), frame_id=("string", str))
Vector3 = _msg("Vector3", x=("double", float), y=("double", float), z=("double", float))
Quaternion = _msg("Quaternion", x=("double", float), y=("double", float), z=("double", float),
                  w=("double", lambda: 1.0))
Point = _msg("Point", x=("double", float), y=("double", float), z=("double", float))
Pose = _msg("Pose", position=("geometry_msgs/Point", Point),
            orientation=("geometry_msgs/Quaternion", Quaternion))
PoseWithCovariance = _msg("PoseWithCovariance", pose=("geometry_msgs/Pose", Pose))
Twist = _msg("Twist", linear=("geometry_msgs/Vector3", Vector3),
             angular=("geometry_msgs/Vector3", Vector3))
TwistWithCovariance = _msg("TwistWithCovariance", twist=("geometry_msgs/Twist", Twist))
JointState = _msg("JointState", header=("std_msgs/Header", Header), name=("sequence<string>", list),
                  position=("sequence<double>", list), velocity=("sequence<double>", list),
                  effort=("sequence<double>", list))
Imu = _msg("Imu", header=("std_msgs/Header", Header),
           orientation=("geometry_msgs/Quaternion", Quaternion),
           angular_velocity=("geometry_msgs/Vector3", Vector3),
           linear_acceleration=("geometry_msgs/Vector3", Vector3))
Odometry = _msg("Odometry", header=("std_msgs/Header", Header), child_frame_id=("string", str),
                pose=("geometry_msgs/PoseWithCovariance", PoseWithCovariance),
                twist=("geometry_msgs/TwistWithCovariance", TwistWithCovariance))
JointTrajectoryPoint = _msg("JointTrajectoryPoint", positions=("sequence<double>", list),
                            velocities=("sequence<double>", list),
                            time_from_start=("builtin_interfaces/Duration", Duration))
JointTrajectory = _msg("JointTrajectory", header=("std_msgs/Header", Header),
                       joint_names=("sequence<string>", list),
                       points=("sequence<trajectory_msgs/JointTrajectoryPoint>", list))
Float64MultiArray = _msg("Float64MultiArray", data=("sequence<double>", list))
# A vendor command whose field names differ from AimRT's JointCommand.
MotorCommand = _msg("MotorCommand", header=("std_msgs/Header", Header),
                    joint=("sequence<string>", list), q=("sequence<double>", list),
                    kp=("sequence<double>", list), kd=("sequence<double>", list),
                    mode=("uint8", int))
# AgiBot X1's my_ros2_proto/JointCommand (agibot_x1_infer).
JointCommand = _msg("JointCommand", header=("std_msgs/Header", Header),
                    name=("sequence<string>", list), position=("sequence<double>", list),
                    velocity=("sequence<double>", list), effort=("sequence<double>", list),
                    stiffness=("sequence<double>", list), damping=("sequence<double>", list))


class Ros:
    """What the fake has seen: subscriptions, publications, nodes, context state."""

    def __init__(self) -> None:
        self.ok = False
        self.inits = 0
        self.subs: Dict[str, List[Any]] = {}
        self.sub_qos: Dict[str, Any] = {}
        self.published: List[tuple] = []
        self.nodes: List[Any] = []
        self.executor_nodes: List[Any] = []
        self._hooks: Dict[str, list] = {}

    def publish_state(self, topic: str, msg: Any) -> None:
        for cb in self.subs.get(topic, []):
            cb(msg)

    def joint_state(self, names, position, velocity=(), effort=(), topic="/joint_states"):
        self.publish_state(topic, JointState(name=list(names), position=list(position),
                                             velocity=list(velocity), effort=list(effort)))

    def imu(self, quat_xyzw=(0.0, 0.0, 0.0, 1.0), topic="/imu/data"):
        x, y, z, w = quat_xyzw
        self.publish_state(topic, Imu(orientation=Quaternion(x=x, y=y, z=z, w=w),
                                      angular_velocity=Vector3(x=0.1),
                                      linear_acceleration=Vector3(z=9.81)))

    def on_subscribe(self, topic: str, fn) -> None:
        """Call ``fn()`` whenever something subscribes to ``topic`` (to publish on connect)."""
        self._hooks.setdefault(topic, []).append(fn)


def install(monkeypatch) -> Ros:
    ros = Ros()

    class Publisher:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, msg):
            ros.published.append((self.topic, msg))

    class Clock:
        def now(self):
            return types.SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=5))

    class Node:
        def __init__(self, name):
            self.name = name
            self.destroyed = False
            ros.nodes.append(self)

        def create_subscription(self, _type, topic, cb, qos):
            ros.subs.setdefault(topic, []).append(cb)
            ros.sub_qos[topic] = qos
            for fn in ros._hooks.get(topic, []):
                fn()

        def create_publisher(self, _type, topic, qos):
            return Publisher(topic)

        def get_clock(self):
            return Clock()

        def destroy_node(self):
            self.destroyed = True

    class SingleThreadedExecutor:
        def add_node(self, node):
            ros.executor_nodes.append(node)

        def remove_node(self, node):
            ros.executor_nodes.remove(node)

        def spin_once(self, timeout_sec=0.0):
            import time

            time.sleep(min(timeout_sec, 0.005))

        def shutdown(self):
            pass

    def init():
        ros.ok = True
        ros.inits += 1

    def shutdown():
        ros.ok = False

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: ros.ok
    rclpy.init = init
    rclpy.shutdown = shutdown
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    executors = types.ModuleType("rclpy.executors")
    executors.SingleThreadedExecutor = SingleThreadedExecutor
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")

    def pkg(name, **classes):
        mod = types.ModuleType(f"{name}.msg")
        for k, v in classes.items():
            setattr(mod, k, v)
        return [(name, types.ModuleType(name)), (f"{name}.msg", mod)]

    modules = [("rclpy", rclpy), ("rclpy.node", node_mod), ("rclpy.executors", executors),
               ("rclpy.qos", qos)]
    modules += pkg("sensor_msgs", JointState=JointState, Imu=Imu)
    modules += pkg("nav_msgs", Odometry=Odometry)
    modules += pkg("trajectory_msgs", JointTrajectory=JointTrajectory,
                   JointTrajectoryPoint=JointTrajectoryPoint)
    modules += pkg("std_msgs", Float64MultiArray=Float64MultiArray, Header=Header)
    modules += pkg("builtin_interfaces", Time=Time, Duration=Duration)
    modules += pkg("geometry_msgs", Vector3=Vector3, Quaternion=Quaternion, Point=Point, Pose=Pose,
                   PoseWithCovariance=PoseWithCovariance, Twist=Twist,
                   TwistWithCovariance=TwistWithCovariance)
    modules += pkg("vendor_msgs", MotorCommand=MotorCommand)
    modules += pkg("my_ros2_proto", JointCommand=JointCommand)
    for name, mod in modules:
        monkeypatch.setitem(sys.modules, name, mod)
    return ros
