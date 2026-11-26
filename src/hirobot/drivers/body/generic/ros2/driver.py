"""A ROS 2 robot from its config alone: topics, message types and field maps in ``driver:``.

Standard messages are components (``joint_state``, ``imu``, ``odometry`` in;
``joint_trajectory``, ``float_array`` out); a custom command message is
``type: message`` with a ``fields`` map. See
:mod:`hirobot.drivers.body.generic.ros2.config` for the schema and
``docs/setup/ros2.md`` for how to write one for a new robot.

Joints are matched by name (``JointState.name``, with ``ros_names`` for names
that differ from the embodiment's), so the publisher's order does not matter.
Callbacks run on one background executor thread shared by every ROS 2 driver
in the process (:mod:`hirobot.drivers._sdk.ros2`); ``get_observation`` reads the
latest message of each topic.

A config is not a calibration: joint names, units, zero offsets and directions,
what the command fields mean, and whether something else on the robot also
publishes commands still have to be checked on each robot.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot import Robot
from hirobot.core.types import Action, Observation
from hirobot.drivers._sdk import ros2 as ros
from hirobot.drivers.body.generic.ros2.components import (
    JointStateComponent,
    make_command,
    make_state,
)
from hirobot.drivers.body.generic.ros2.config import Ros2Config
from hirobot.loading.registry import register_body
from hirobot.loading.sdk import require_sdk

WHO = "Ros2Robot"


@register_body("ros2")
class Ros2Robot(Robot):
    """Generic ROS 2 body driver.

    Args:
        embodiment: The robot's embodiment; its joint names (or ``ros_names``)
            must match the names in the robot's ``JointState``.
        read_only: Create no publisher; commands raise. ``hirobot view`` sets it.
        **config: The rest of the ``driver:`` section without ``type``
            (:class:`~hirobot.drivers.body.generic.ros2.config.Ros2Config`).
            It is checked here, so a bad config fails before anything connects.
    """

    def __init__(self, embodiment: Embodiment, read_only: bool = False, **config: Any) -> None:
        self._embodiment = embodiment
        self._config = Ros2Config.model_validate(dict(config, read_only=read_only))
        names = embodiment.joint_names
        n = len(names)
        unknown = sorted(set(self._config.ros_names) - set(names))
        if unknown:
            raise ValueError(f"{WHO}: ros_names has joints not in the embodiment: {unknown}")
        self._state = [make_state(c, names, self._config.ros_names) for c in self._config.state]
        self._joint_states = [s for s in self._state if isinstance(s, JointStateComponent)]
        self._check_joint_sources(names)
        self._command = ([] if self._config.read_only else
                         [make_command(c, names, self._config.ros_names)
                          for c in self._config.command])
        modes = [] if self._config.read_only else self._config.modes
        self.action_modes = tuple(dict.fromkeys(modes))
        kp, kd = self._config.default_kp, self._config.default_kd
        # None: no default; a command that needs gains must then carry them.
        self._kp = None if kp is None else np.full(n, float(kp))
        self._kd = None if kd is None else np.full(n, float(kd))
        self._rclpy: Any = None
        self._node: Any = None

    def _check_joint_sources(self, names: List[str]) -> None:
        count = np.zeros(len(names), dtype=int)
        for comp in self._joint_states:
            for i, _ in comp.joints:
                count[i] += 1
        twice = [names[i] for i in np.flatnonzero(count > 1)]
        none = [names[i] for i in np.flatnonzero(count == 0)]
        if twice:
            raise ValueError(f"{WHO}: joints in more than one joint_state component: {twice}")
        if none:
            raise ValueError(f"{WHO}: joints in no joint_state component: {none}")

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    @property
    def read_only(self) -> bool:
        return self._config.read_only

    # ------------------------------------------------------------ lifecycle
    def connect(self) -> None:
        if self._node is not None:
            return
        require_sdk(__package__, WHO)
        rclpy = ros.acquire(WHO)
        try:
            from rclpy.node import Node

            node = Node(self._config.node_name)
            for comp in self._state:
                comp.connect(node)
            for comp in self._command:
                comp.connect(node)
            ros.add_node(node)
        except BaseException:
            ros.release(locals().get("node"))
            raise
        self._rclpy, self._node = rclpy, node
        try:
            self._wait_first_messages()
        except BaseException:
            self.disconnect()
            raise

    def _wait_first_messages(self) -> None:
        deadline = time.monotonic() + self._config.connect_timeout_s
        while True:
            waiting = [c.topic for c in self._state if c.latest()[0] is None]
            missing = {c.topic: c.missing() for c in self._joint_states if c.latest()[0] is not None}
            missing = {t: m for t, m in missing.items() if m}
            if not waiting and not missing:
                return
            if time.monotonic() > deadline:
                if waiting:
                    raise TimeoutError(
                        f"{WHO}: nothing on {waiting} in {self._config.connect_timeout_s}s. "
                        "Check the robot side is up (ros2 topic echo <topic>), the ROS_DOMAIN_ID, "
                        "and that the topic's QoS matches (state defaults to best_effort).")
                raise RuntimeError(
                    f"{WHO}: JointState on {list(missing)} lacks joints "
                    f"{sorted(j for m in missing.values() for j in m)}; fix the embodiment's "
                    "joint names or set ros_names.")
            time.sleep(0.01)

    def disconnect(self) -> None:
        if self._node is None:
            return
        for comp in self._command:
            comp.disconnect()
        node, self._node = self._node, None
        ros.release(node)
        self._rclpy = None

    def reset(self, seed: Optional[int] = None) -> None:
        """Nothing to do: the robot side owns its modes and homing."""
        self._require_connected()

    def _require_connected(self) -> None:
        if self._node is None:
            raise RuntimeError(f"{WHO}: not connected; call connect() first.")

    # ------------------------------------------------------------ I/O
    def get_observation(self) -> Observation:
        self._require_connected()
        n = self._embodiment.num_dofs
        pos, vel, eff = np.zeros(n), np.zeros(n), np.zeros(n)
        all_vel = all_eff = True
        stamp = None
        for comp in self._joint_states:
            has_vel, has_eff = comp.read_into(pos, vel, eff)
            all_vel &= has_vel
            all_eff &= has_eff
            s = comp.latest()[1]
            # The oldest joint reading stamps the body, so its age is never understated.
            if stamp is None or s.t < stamp.t:
                stamp = s
        obs = Observation(joint_pos=pos, joint_vel=vel if all_vel else None,
                          joint_torque=eff if all_eff else None)
        for comp in self._state:
            if not isinstance(comp, JointStateComponent):
                comp.read(obs)
        obs.stamps["body"] = stamp
        return obs

    def _send_action(self, action: Action) -> None:
        if self._config.read_only:
            raise RuntimeError(f"{WHO} is read_only")
        self._require_connected()
        comp = next((c for c in self._command if action.mode in c.modes), None)
        if comp is None:
            raise ValueError(f"{WHO}: no command component for mode {action.mode.value!r}; "
                             f"this robot takes {[m.value for m in self.action_modes]}")
        if action.joint_target is None:
            return
        kp, kd = self._gains(action, comp)
        comp.publish(action, kp, kd)

    def _gains(self, action: Action, comp: Any) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        n = self._embodiment.num_dofs
        kp = self._kp if action.kp is None else np.broadcast_to(np.asarray(action.kp, float), (n,))
        kd = self._kd if action.kd is None else np.broadcast_to(np.asarray(action.kd, float), (n,))
        need = {v for v in getattr(comp, "fields", {}).values() if isinstance(v, str)}
        for src, value in (("$kp", kp), ("$kd", kd)):
            if src in need and value is None:
                raise ValueError(f"{WHO}: {comp.topic} needs {src[1:]} but the action carries "
                                 f"none and the config sets no default_{src[1:]}")
        return kp, kd

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        n = self._embodiment.num_dofs
        if kp is not None:
            self._kp = np.array(np.broadcast_to(np.asarray(kp, float), (n,)))
        if kd is not None:
            self._kd = np.array(np.broadcast_to(np.asarray(kd, float), (n,)))

    def describe(self) -> Dict[str, Any]:
        """The topics this driver uses, for ``hirobot info`` and error reports."""
        return {"node": self._config.node_name,
                "state": {c.topic: c.msg_type for c in self._state},
                "command": {c.topic: [m.value for m in c.modes] for c in self._command}}
