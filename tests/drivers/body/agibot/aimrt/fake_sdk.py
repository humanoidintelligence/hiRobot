"""Fake ROS 2 for ``agibot_aimrt``: ``rclpy``, ``sensor_msgs``, and the AimRT ``JointCommand``.

``spin_once`` delivers the queued ``/joint_states`` and ``/imu/data`` messages to
the node's subscriptions, so the driver sees state the way it would from ROS.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace


class Ros:
    def __init__(self):
        self.ok = False
        self.subs = {}
        self.published = []
        self.pending = []  # (topic, msg) delivered on the next spin_once
        self.destroyed = False

    def joint_state(self, position, velocity=(), effort=()):
        msg = SimpleNamespace(position=list(position), velocity=list(velocity), effort=list(effort))
        self.pending.append(("/joint_states", msg))

    def imu(self, quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
        x, y, z, w = quat_xyzw
        msg = SimpleNamespace(orientation=SimpleNamespace(x=x, y=y, z=z, w=w),
                              angular_velocity=SimpleNamespace(x=0.1, y=0.0, z=0.0),
                              linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.81))
        self.pending.append(("/imu/data", msg))


def install(monkeypatch):
    ros = Ros()

    class Publisher:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, msg):
            ros.published.append((self.topic, msg))

    class Node:
        def __init__(self, name):
            self.name = name

        def create_publisher(self, _type, topic, _depth):
            return Publisher(topic)

        def create_subscription(self, _type, topic, cb, _depth):
            ros.subs[topic] = cb

        def destroy_node(self):
            ros.destroyed = True

    def spin_once(node, timeout_sec=0.0):
        pending, ros.pending = ros.pending, []
        for topic, msg in pending:
            if topic in ros.subs:
                ros.subs[topic](msg)

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: ros.ok
    rclpy.init = lambda: setattr(ros, "ok", True)
    rclpy.shutdown = lambda: setattr(ros, "ok", False)
    rclpy.spin_once = spin_once
    node = types.ModuleType("rclpy.node")
    node.Node = Node
    sensor = types.ModuleType("sensor_msgs.msg")
    sensor.Imu, sensor.JointState = object, object
    proto = types.ModuleType("my_ros2_proto.msg")
    proto.JointCommand = lambda: SimpleNamespace()
    for name, mod in [("rclpy", rclpy), ("rclpy.node", node),
                      ("sensor_msgs", types.ModuleType("sensor_msgs")), ("sensor_msgs.msg", sensor),
                      ("my_ros2_proto", types.ModuleType("my_ros2_proto")),
                      ("my_ros2_proto.msg", proto)]:
        monkeypatch.setitem(sys.modules, name, mod)
    return ros
