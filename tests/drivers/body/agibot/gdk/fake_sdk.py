"""Fake ``cosine_bus.agibotdds`` + GDK messages for ``agibot_gdk``, and a fake chassis box.

Subscribers get the robot's state as soon as they are created (the real HAL
publishes continuously); a test changes it with :meth:`Gdk.set` and
:meth:`Gdk.publish_state`. Published commands land in :attr:`Gdk.published`.
A SERVO request on ``/wbc/set_control_mode`` switches the fake controller's mode.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace as NS
from typing import Any, Dict, List, Tuple

from hirobot.drivers.body.agibot.gdk.driver import ARM_JOINTS

ARM_POS = [-0.71, 1.58, 0.66, -1.15, 1.96, 1.21, 0.21,
           1.0, -1.40, -0.83, 1.05, -1.91, -0.48, 0.24]


def _stamp():
    return NS(header=NS(stamp=NS(sec=0, nanosec=0)))


def _motor(pos, err=0):
    return NS(position=pos, velocity=0.0, effort=0.0, err_code=err)


class _Msg:
    """A protobuf-like message: attributes, repeated fields as lists."""

    _repeated: Tuple[str, ...] = ()

    def __init__(self):
        for name in self._repeated:
            setattr(self, name, [])


class JointPositionControl(_Msg):
    _repeated = ("left_arm_joint_positions", "right_arm_joint_positions",
                 "left_tool_joint_positions", "right_tool_joint_positions")


class SetControlMode(_Msg):
    pass


class MotionStop(_Msg):
    pass


class Gdk:
    def __init__(self):
        self.subs: Dict[str, Any] = {}
        self.published: List[Tuple[str, Any]] = []
        self.mode = 1
        self.arm_pos = list(ARM_POS)
        self.waist = {"joint_lift_body": 0.289, "joint_body_pitch": 0.401}
        self.head = {"joint_head_yaw": 0.0, "joint_head_pitch": 0.436}
        self.grippers = {"left": 120.0, "right": 0.0}
        self.motion_error = 0
        self.estop = False
        self.servo_on_request = True

    def messages(self) -> Dict[str, Any]:
        arms = NS(name=list(ARM_JOINTS), position=list(self.arm_pos),
                  velocity=[0.0] * 14, effort=[], **vars(_stamp()))
        waist = NS(name=list(self.waist), motor_states=[_motor(v) for v in self.waist.values()])
        head = NS(name=list(self.head),
                  motor_states=[_motor(v) for v in self.head.values()])
        ee = {side: NS(type=2, name=[f"{side}_gripper_joint1"],
                       end_state=[_motor(self.grippers[side])])
              for side in ("left", "right")}
        pose = NS(position=NS(x=0.46, y=-0.05, z=0.49),
                  orientation=NS(x=0.23, y=0.85, z=-0.47, w=-0.01))
        motion = NS(mode=self.mode, error_code=self.motion_error, error_msg="",
                    frame_names=["arm_left_link7", "arm_right_link7"], frame_poses=[pose, pose])
        body_fields = dict(left_arm_error=0, right_arm_error=0, left_arm_estop=self.estop,
                           right_arm_estop=False, neck_error=4294967295)
        body = NS(DESCRIPTOR=NS(fields=[NS(name=k) for k in ("header", *body_fields)]),
                  **body_fields)
        return {"/hal/arm_joint_state": arms, "/hal/left_ee_data": ee["left"],
                "/hal/right_ee_data": ee["right"], "/hal/waist_state": waist,
                "/hal/neck_state": head, "/hal/whole_body_status": body,
                "/wbc/motion_control_status": motion}

    def publish_state(self) -> None:
        msgs = self.messages()
        for topic, callback in self.subs.items():
            callback(msgs[topic])

    def joint_commands(self) -> List[Any]:
        return [m for t, m in self.published if t == "/wbc/joint_position_control"]


def install(monkeypatch, start_mode: int = 1) -> Gdk:
    gdk = Gdk()
    gdk.mode = start_mode

    class Publisher:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, msg):
            gdk.published.append((self.topic, msg))
            if self.topic == "/wbc/set_control_mode" and gdk.servo_on_request:
                gdk.mode = msg.control_mode
                gdk.publish_state()
            elif self.topic == "/wbc/motion_stop":
                gdk.mode = 0
                gdk.publish_state()

    class Node:
        def __init__(self, name):
            self.name = name

        def create_subscriber(self, topic, msg_type, qos, callback):
            gdk.subs[topic] = callback
            callback(gdk.messages()[topic])
            return NS(topic=topic)

        def create_publisher(self, topic, msg_type, qos):
            return Publisher(topic)

    agibotdds = types.ModuleType("agibotdds")
    agibotdds.Node = Node
    agibotdds.qos = lambda depth=5: NS(depth=depth)
    modules = {
        "cosine_bus": types.ModuleType("cosine_bus"),
        "cosine_bus.agibotdds_py3": types.ModuleType("cosine_bus.agibotdds_py3"),
        "genie_msgs_pb": types.ModuleType("genie_msgs_pb"),
        "genie_msgs_pb.msg": types.ModuleType("genie_msgs_pb.msg"),
        "sensor_msgs_pb": types.ModuleType("sensor_msgs_pb"),
        "sensor_msgs_pb.msg": types.ModuleType("sensor_msgs_pb.msg"),
        # The chassis client (websockets) is replaced by install_chassis; this
        # stub only lets require_sdk find it where websockets is not installed.
        "websockets": types.ModuleType("websockets"),
    }
    modules["cosine_bus.agibotdds_py3"].agibotdds = agibotdds
    for name in ("EndState", "HeadState", "WaistState", "MotionControlStatus", "WholeBodyStatus",
                 "JointPositionControl", "MotionStop", "SetControlMode"):
        mod = types.ModuleType(f"genie_msgs_pb.msg.{name}_pb2")
        setattr(mod, name, globals().get(name, type(name, (_Msg,), {})))
        modules[mod.__name__] = mod
    js = types.ModuleType("sensor_msgs_pb.msg.JointState_pb2")
    js.JointState = type("JointState", (_Msg,), {})
    modules[js.__name__] = js
    for name, mod in modules.items():
        monkeypatch.setitem(sys.modules, name, mod)
    return gdk


class Chassis:
    """Stand-in for :class:`~hirobot.drivers.body.agibot.gdk.chassis.ChassisClient`."""

    def __init__(self, soft_manual: bool = True):
        self.moves: List[Tuple[float, float]] = []
        self.closed = False
        self.state = {"odometer": {"x": 1000, "y": -500, "theta": 90.0},
                      "isSoftAuto": not soft_manual, "isHardAuto": soft_manual,
                      "isScram": False, "exceptions": [0, 0, 0],
                      "batteryStateList": [{"energy": 100, "voltage": 56.9}]}

    def get_state(self):
        return dict(self.state)

    def move(self, linear_mm_s, angular_deg_s):
        self.moves.append((linear_mm_s, angular_deg_s))

    def close(self):
        self.closed = True


def install_chassis(monkeypatch, chassis: Chassis) -> None:
    import hirobot.drivers.body.agibot.gdk.driver as drv

    monkeypatch.setattr(drv, "ChassisClient", lambda host, robot_id: chassis)
