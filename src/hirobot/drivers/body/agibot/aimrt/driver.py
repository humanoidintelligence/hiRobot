"""AgiBot X1 humanoid driver (ROS2 / AimRT bridge).

Unlike the other vendors, ``agibot_x1_infer`` has **no Python API** — it is a
C++/AimRT stack. The only realistic way to drive X1 from hiRobot is as a ROS2
node that talks to the AimRT process over its ``ros2`` channel backend:

* subscribe to ``/joint_states`` (``sensor_msgs/JointState``) and ``/imu/data``
  (``sensor_msgs/Imu``) for observations;
* publish ``/joint_cmd`` (``my_ros2_proto/JointCommand``: name / position /
  velocity / effort / stiffness / damping) for actions, where ``stiffness`` /
  ``damping`` are the PD gains (kp / kd).

Three setup prerequisites must be satisfied on the robot side (see
``docs/setup/agibot_x1.md``):

1. The AimRT config's ``sub_topics_options`` must enable the ``ros2`` backend
   (it defaults to ``local`` only, so external ``/joint_cmd`` never arrives).
2. The running package must expose the low-level driver (``DcuDriverModule`` on
   hardware, ``SimModule`` in MuJoCo) and **not** the built-in ``ControlModule``,
   which would otherwise also publish ``/joint_cmd`` and fight for control.
3. The custom ``my_ros2_proto`` messages must be built and importable from
   Python (``colcon build`` that ROS2 package).

Because of this, the driver is a thin rclpy bridge that **fails loudly** at
``connect()`` if rclpy or the custom message is unavailable, pointing to the
setup doc. The observation/action mapping itself is complete, so once the
ROS2 side is up it is ready to drive.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.stamps import StampCounter
from hirobot.sdk import require_sdk
from hirobot.types import Action, ControlMode, Observation

_JOINT_STATES_TOPIC = "/joint_states"
_IMU_TOPIC = "/imu/data"
_JOINT_CMD_TOPIC = "/joint_cmd"


@register_body("agibot_aimrt")
class AgibotX1Robot(Robot):
    """ROS2-bridge driver for the AgiBot X1 humanoid.

    Args:
        embodiment: The X1 embodiment; its joint names must match the names the
            AimRT stack uses, since ``/joint_cmd`` matches by name, not index.
        node_name: Name for the rclpy node this driver creates.
        default_kp / default_kd: Fallback PD gains (stiffness/damping) used when
            an action does not carry its own.
        spin_timeout: Seconds to wait for the first ``/joint_states`` on connect.
    """

    action_modes = (ControlMode.POSITION, ControlMode.PD)

    def __init__(
        self,
        embodiment: Embodiment,
        node_name: str = "hirobot_agibot_x1",
        default_kp: float = 100.0,
        default_kd: float = 2.0,
        spin_timeout: float = 5.0,
    ) -> None:
        self._embodiment = embodiment
        self._node_name = node_name
        self._n = embodiment.num_dofs
        self._kp = np.full(self._n, default_kp, dtype=float)
        self._kd = np.full(self._n, default_kd, dtype=float)
        self._spin_timeout = spin_timeout
        self._stamps = StampCounter()

        self._rclpy = None
        self._node = None
        self._pub = None
        self._JointCommand = None
        self._latest_joint_state = None
        self._joint_state_stamp = None
        self._latest_imu = None

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def connect(self) -> None:
        require_sdk(__package__, "AgibotX1Robot")
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Imu, JointState
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "AgibotX1Robot needs a ROS2 Python environment (rclpy, "
                "sensor_msgs). See docs/setup/agibot_x1.md."
            ) from exc
        try:
            # Custom AimRT message; must be built + sourced (prerequisite 3).
            from my_ros2_proto.msg import JointCommand
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "AgibotX1Robot needs the my_ros2_proto messages built and "
                "sourced (colcon build). See docs/setup/agibot_x1.md."
            ) from exc

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._JointCommand = JointCommand
        self._node = Node(self._node_name)
        self._pub = self._node.create_publisher(JointCommand, _JOINT_CMD_TOPIC, 10)
        self._node.create_subscription(
            JointState, _JOINT_STATES_TOPIC, self._on_joint_state, 10
        )
        self._node.create_subscription(Imu, _IMU_TOPIC, self._on_imu, 10)

        # Wait for the first joint state so get_observation has data.
        deadline = self._spin_timeout
        dt = 0.02
        while self._latest_joint_state is None and deadline > 0:
            rclpy.spin_once(self._node, timeout_sec=dt)
            deadline -= dt
        if self._latest_joint_state is None:
            raise TimeoutError(
                f"No {_JOINT_STATES_TOPIC} received in {self._spin_timeout}s. "
                "Check the AimRT process is up and ros2 backend is enabled "
                "(see docs/setup/agibot_x1.md)."
            )

    def _on_joint_state(self, msg) -> None:
        self._joint_state_stamp = self._stamps.tick()
        self._latest_joint_state = msg

    def _on_imu(self, msg) -> None:
        self._latest_imu = msg

    def reset(self, seed: Optional[int] = None) -> None:
        """No-op: X1 setup (modes, zeroing) is driven by the AimRT stack."""

    def _spin(self) -> None:
        if self._node is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def get_observation(self) -> Observation:
        if self._node is None:
            raise RuntimeError("Not connected; call connect() first.")
        self._spin()
        js = self._latest_joint_state
        if js is None:
            raise RuntimeError("No joint state received yet.")
        obs = Observation(
            joint_pos=np.asarray(js.position, dtype=float),
            joint_vel=np.asarray(js.velocity, dtype=float) if len(js.velocity) else None,
            joint_torque=np.asarray(js.effort, dtype=float) if len(js.effort) else None,
        )
        if self._embodiment.floating_base and self._latest_imu is not None:
            o = self._latest_imu.orientation
            w = self._latest_imu.angular_velocity
            a = self._latest_imu.linear_acceleration
            obs.base_orientation = np.array([o.x, o.y, o.z, o.w])
            obs.base_angular_velocity = np.array([w.x, w.y, w.z])
            obs.base_linear_acceleration = np.array([a.x, a.y, a.z])
        if self._joint_state_stamp is not None:
            obs.stamps["body"] = self._joint_state_stamp
        return obs

    def _send_action(self, action: Action) -> None:
        if self._pub is None:
            raise RuntimeError("Not connected; call connect() first.")
        if action.mode not in (ControlMode.POSITION, ControlMode.PD):
            raise ValueError(
                f"AgibotX1Robot supports POSITION/PD (stiffness/damping); got "
                f"{action.mode!r}."
            )
        if action.joint_target is None:
            return
        kp = self._kp if action.kp is None else np.asarray(action.kp, dtype=float)
        kd = self._kd if action.kd is None else np.asarray(action.kd, dtype=float)

        cmd = self._JointCommand()
        cmd.name = list(self._embodiment.joint_names)
        cmd.position = [float(x) for x in np.asarray(action.joint_target, dtype=float)]
        cmd.velocity = []
        cmd.effort = []
        cmd.stiffness = [float(x) for x in np.broadcast_to(kp, (self._n,))]
        cmd.damping = [float(x) for x in np.broadcast_to(kd, (self._n,))]
        self._pub.publish(cmd)
        self._spin()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if kp is not None:
            kp = np.asarray(kp, dtype=float)
            self._kp = np.full(self._n, float(kp)) if kp.ndim == 0 else kp
        if kd is not None:
            kd = np.asarray(kd, dtype=float)
            self._kd = np.full(self._n, float(kd)) if kd.ndim == 0 else kd

    def disconnect(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
            self._rclpy = None
